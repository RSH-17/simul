"""Choose an entrant using only the five original, paired 8780 development tracks."""
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ADAPTIVE=HERE.parent/'2026-11-22_v151_adaptive_survey'
R4_TOOLS=HERE.parent/'2026-11-22_v151_tiny_named_relook'
sys.path[:0]=[str(R4_TOOLS),str(ADAPTIVE/'development'),str(ADAPTIVE/'validation'),str(ADAPTIVE)]
from track_screen import ORDER, judge

HOME=Path.home()
R2=HOME/'v151-adaptive-survey-paired-1122'
R3=HOME/'v151-object-followthrough-screen-112203'
R4=HOME/'v151-tiny-named-relook-track-112204'
ARCHIVE=HOME/'v151-track-five-boxes5-1795247134501974510'


def read(path):
    return json.loads(path.read_text())


def stable(data):
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def normalized(rows, expected, source):
    seeds=[p['seed'] for p in rows]
    if len(seeds)!=len(set(seeds)) or set(seeds)!=set(expected):
        raise ValueError('Missing or duplicate original-track evidence')
    ordered=sorted(rows,key=lambda p:ORDER.index(p['seed']))
    for p in ordered:
        for role in ('baseline','candidate'):
            r=p[role]
            ids=set(r['registered_ids']);near=set(r['near_ids'])
            walls=set(r['wall_ids']);boxes=set(r['box_ids'])
            times={int(i):v for i,v in r['raw_near_time_s'].items()}
            if (not r['valid'] or len(walls)!=1 or len(boxes)!=2 or walls&boxes or walls|boxes!={7,32,103}
                    or not near.issubset(times) or not near.issubset(walls|boxes)
                    or any(not math.isfinite(t) or not 0<=t<=600 for t in times.values())
                    or r['world_sha256']!=expected[p['seed']] or len(r['settled_xy'])!=2
                    or len(r['configured_start'])!=3 or not all(map(math.isfinite,r['configured_start']+r['settled_xy']))
                    or not math.isfinite(r['elapsed_s']) or r['elapsed_s']<0
                    or type(r['strict']) is not bool or r['strict']!=(r['original_strict'] and r['raw_strict'])
                    or (r['strict'] and (len(near)!=3 or r['elapsed_s']>600 or r['unexpected_ids'] or r['bumps']))):
                raise ValueError(f"{p['seed']} {role}: invalid raw evidence")
            for field,hosts in [('wall',walls),('box',boxes)]:
                if r[field+'_reg']!=len(ids & hosts) or r[field+'_near']!=len(near & hosts):
                    raise ValueError('Counts disagree with raw marker sets')
            if role=='candidate' and r['run_source_sha256']!=source:
                raise ValueError('Different candidate source')
    return ordered


def adjudicate(revision, rows, expected, source, futility):
    ordered=normalized(rows,expected,source)
    decision=judge(ordered,not futility)
    if decision['state']=='hold' or (futility and decision['state']!='declined'):
        raise ValueError('Terminal decision and independent track evidence disagree')
    legs=[p['candidate'] for p in ordered]
    return dict(revision=revision,state='declined' if futility else decision['state'],futility=futility,
        paired=len(ordered),strict=sum(r['strict'] for r in legs),verified_near=sum(len(r['near_ids']) for r in legs),
        capped_mean_s=sum(r['elapsed_s'] if r['strict'] else 600. for r in legs)/len(legs),
        source_manifest_sha256=source,decision=decision,pairs=ordered)


def choose(a,b):
    if {a['revision'],b['revision']}!={'R3','R4'}:
        raise ValueError('Missing distinct engineering revisions')
    if any(x['state'] not in ('passed','declined') for x in (a,b)):
        return dict(mode='wait',reason='Both revisions need a valid terminal policy decision')
    shared={p['seed']:p['baseline'] for p in a['pairs']}
    for p in b['pairs']:
        if p['seed'] in shared:
            lhs,rhs=shared[p['seed']],p['baseline']
            keys=('run','world_sha256','configured_start','settled_xy','wall_ids','box_ids','registered_ids','near_ids','strict','bumps','elapsed_s')
            if any(lhs[k]!=rhs[k] for k in keys):
                raise ValueError('Different baseline data for the same development track')
    passed=[x for x in (a,b) if x['state']=='passed']
    if not passed:
        return dict(mode='anchor_only',revision='R4',required_pairs=2,development_qualified=False,auto_promote=False)
    winner=max(passed,key=lambda r:(r['strict'],r['verified_near'],-r['capped_mean_s'],r['revision']=='R3'))
    return dict(mode='full',revision=winner['revision'],required_pairs=9,development_qualified=True,auto_promote=False)


def raw_r3(seed, old, role, world, identity):
    # Reuse the exact development audit's allowlist, per-run source SHA and raw
    # time-aligned scorer. Inspect only original large-track recordings.
    import screen_audit as raw
    run=raw.run_path('archive' if role=='baseline' else 'candidate','R3',seed,old['run'])
    metric=read(run/'metrics.json')
    check=raw.measure(run,{'files':{'world/osp_studio.sdf':world}})
    src=run/'policy_manifest.json'
    source=raw.checksum(read(src)) if src.exists() else None
    if (metric['world_sha256']!=old['sha256'] or sorted(metric['final_ids'])!=sorted(old['final_ids'])
            or sorted(map(int,metric['registered_and_within_s']))!=old['verified_near_ids']
            or old['elapsed_s']!=metric['elapsed_sim_s']
            or source!=identity['source_manifest_sha256'] and (source is not None or role=='candidate')):
        raise ValueError('R3 cached result and native recording differ')
    ids=set(check['registered_ids']) & set(metric['final_ids'])
    near=set(check['after_registration_near_s']) & set(old['verified_near_ids'])
    out=dict(run=run.name,world_sha256=metric['world_sha256'],configured_start=old['configured_start'],
             settled_xy=check['settled_xy'],valid=check['valid'],wall_ids=old['wall_ids'],box_ids=old['box_ids'],
             strict=old['strict'] and check['strict'],original_strict=old['strict'],raw_strict=check['strict'],
             registered_ids=sorted(ids),near_ids=sorted(near),raw_near_time_s=check['after_registration_near_s'],
             elapsed_s=check['elapsed_s'],bumps=check['bumps'],unexpected_ids=check['unexpected_ids'],run_source_sha256=source)
    for tag in ('box','wall'):
        hosts=set(out[tag+'_ids'])
        out[tag+'_reg']=len(ids & hosts)
        out[tag+'_near']=len(near & hosts)
    return out


def select(root):
    queue=HOME/'v151-object-followthrough-queue1122.json'
    if not queue.exists() or read(queue).get('state') not in {'futility_stop','complete','skipped_r2_eligible'}:
        return dict(mode='wait',reason='preceding engineering queue is unfinished or requires investigation')
    if not (R4/'terminal.json').exists():
        return dict(mode='wait',reason='R4 has no terminal development evidence')
    terminal=read(R4/'terminal.json')
    if terminal['state']=='infrastructure_hold':
        return dict(mode='hold',reason='R4 development infrastructure hold')
    if terminal['state'] not in ('five_track_screen_passed','screen_declined'):
        return dict(mode='hold',reason='unknown R4 terminal state')
    early3=(R3/'futility_stop.json').is_file()
    if not early3 and not (R3/'candidate/complete.json').is_file():
        return dict(mode='wait',reason='R3 original-track measurements are missing')
    early4=terminal['count']==2
    if (early3 and read(R3/'futility_stop.json').get('finished_cases')!=2 or
            terminal['count'] not in (2,5) or early4 and terminal['state']!='screen_declined'):
        raise ValueError('Invalid terminal episode counts')
    proto=read(root/'protocol.json');frozen=proto['policies']
    if proto['physical_drive'] is not False or proto['domain']!=174:
        raise ValueError('Unexpected official protocol')
    from validate import identity
    for role in ('baseline','R3','R4'):
        if identity(Path(frozen[role]['workspace']))!=frozen[role]:
            raise ValueError('Official policy identity differs: '+role)
    stage=read(R4/'registration.json')
    if stage['approval']!='2c017477e7d20ca0894cb61b0c92ef12238c918740cf3c5dc5568408e5e4c1de':
        raise ValueError('R4 screen lacks the reviewed replay/Jazzy approval receipt')
    registration=stage['source']
    if (registration['workspace_identity_sha256']!=stable(frozen['R4'])
            or registration['baseline_identity_sha256']!=stable(frozen['baseline'])):
        raise ValueError('R4 screen and official policy installations differ')
    expected=read(R2/'protocol.json')['worlds']
    s3=ORDER[:2] if early3 else ORDER
    s4=ORDER[:2] if early4 else ORDER
    def worlds(seeds):return {s:expected[str(s)] for s in seeds}
    rows=[]
    for pair in read(R3/'paired_comparison.json')['regression']['pairs']:
        seed=pair['seed']
        if seed not in s3:
            continue
        rows.append(dict(seed=seed,**{r:raw_r3(seed,pair[r],r,expected[str(seed)],frozen['baseline' if r=='baseline' else 'R3'])
                                      for r in ('baseline','candidate')}))
    a=adjudicate('R3',rows,worlds(s3),frozen['R3']['source_manifest_sha256'],early3)
    data=read(R4/'track_comparison.json')
    b=adjudicate('R4',data['pairs'],worlds(s4),frozen['R4']['source_manifest_sha256'],early4)
    if (b['state']=='passed')!=(terminal['state']=='five_track_screen_passed'):
        raise ValueError('R4 track terminal state conflicts with raw report')
    return dict(**choose(a,b),protocol_sha256=hashlib.sha256((root/'protocol.json').read_bytes()).hexdigest(),
                candidates={'R3':a,'R4':b},scope='only original five tracks; archived balanced has no per-run policy digest')
