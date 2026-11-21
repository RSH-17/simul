"""Run or inspect a pre-enrolled native exam-track comparison, entirely in simulation."""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys

HERE = Path(__file__).resolve().parent
ENGINEERING = HERE.parent/'2026-11-22_v151_adaptive_survey'
REQUIRED = {'world/osp_studio.sdf','scene.json','scene.yaml',
            'run/simulation/demo_layout.json','run/simulation/demo_floor.png'}
ANCHOR = 2983619014
XY_TOL, YAW_TOL = .15, math.radians(5)
BOARD = 'http://192.168.252.2:8780'


def read(p):
    return json.loads(p.read_text())


def save(p,obj):
    tmp=p.with_suffix('.partial')
    tmp.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(p)


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def fingerprint(obj):
    return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def utc():
    return datetime.now(timezone.utc).isoformat()


def plan(protocol,revision,anchor_only=False):
    if (revision not in ('R3','R4') or protocol['physical_drive'] is not False or protocol['domain']!=174
            or protocol['camera']!=[480,480,5] or protocol['duration_s']!=600 or protocol['anchor_seed']!=ANCHOR):
        raise ValueError('Expected enrolled Jazzy simulation, domain 174, 480x480 at 5 fps and 600 s')
    nonce=protocol['nonce']
    if not isinstance(nonce,str) or len(nonce)<8 or any(c not in '0123456789abcdefABCDEF' for c in nonce):
        raise ValueError('Invalid experiment nonce')
    if not all(k in protocol['policies'] for k in ('baseline',revision)):
        raise ValueError('Requested frozen policy sources missing')
    worlds={w['seed']:w for w in protocol['worlds']}
    if len(worlds)!=len(protocol['worlds']) or ANCHOR not in worlds:
        raise ValueError('Missing anchor or duplicate world seed')
    for w in worlds.values():
        if (set(w['files'])!=REQUIRED or len(w['start'])!=3 or not all(map(math.isfinite,w['start']))
                or type(w['boxes']) is not int or w['boxes'] < 0):
            raise ValueError('Incomplete exam-world identity')
    tasks=protocol['plan']
    if (not tasks or len({t['pair'] for t in tasks})!=len(tasks)
            or len({(t['seed'],t['repeat']) for t in tasks})!=len(tasks)
            or any(t['seed'] not in worlds or t['order'] not in
                   (['baseline','candidate'],['candidate','baseline']) for t in tasks)):
        raise ValueError('Invalid enrolled pair ordering')
    anchors=[t for t in tasks if t['seed']==ANCHOR]
    if len(anchors)<2:raise ValueError('Two enrolled anchor pairs required')
    return (anchors[:2] if anchor_only else tasks),worlds


def tag(protocol,revision,pair):
    data=f"{protocol['nonce']}:{revision}:{pair}".encode()
    return str(int(hashlib.sha256(data).hexdigest()[:15],16))


def errors(a,b):
    return math.dist(a[:2],b[:2]),abs(math.remainder(a[2]-b[2],math.tau))


def start_pose(run):
    with (run/'headings.csv').open() as f:
        for r in csv.DictReader(f):
            t=float(r['elapsed_sim_s'])
            if t<0:continue
            p=[float(r[k]) for k in ('x','y','yaw_rad')]
            if t<=.5 and all(map(math.isfinite,p)):return t,p
            break
    raise ValueError('Missing finite initial world x,y,yaw within 0.5 simulation seconds')


def checked_files(folder,world):
    for name,expected in world['files'].items():
        if sha(folder/name)!=expected:raise ValueError('Changed exam fixture: '+str(folder/name))


def execution_files():
    return (HERE/'prepare.py',HERE/'paired.py',ENGINEERING/'run.py',ENGINEERING/'trial.py',
            ENGINEERING/'validation/validate.py')


def verify_inputs(root,proto,revision,world,identity):
    record=read(root/'enrollment.json')
    if any(proto.get(k)!=v for k,v in record.items()):
        raise ValueError('Protocol differs from pre-outcome enrollment')
    if proto.get('execution')!={p.name:sha(p) for p in execution_files()}:
        raise ValueError('Frozen native runner or scoring source changed')
    checked_files(root/'fixtures'/f"map-{world['seed']}",world)
    import yaml
    scene=yaml.safe_load((root/'fixtures'/f"map-{world['seed']}"/'scene.yaml').read_text())
    if list(scene['start'])!=list(world['start']):raise ValueError('Launch pose differs from enrolled pose')
    for name in ('baseline',revision):
        enrolled=proto['policies'][name]
        if identity(Path(enrolled['workspace']))!=enrolled:
            raise ValueError('Frozen policy, installed package or runtime helper changed: '+name)


def after_start_ids(run,expected):
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines() if line.strip()]
    starts=[e for e in events if e['event']=='start_ack']
    if len(starts)!=1:raise ValueError('Missing or ambiguous start acknowledgement')
    start=starts[0]['sim']
    false=set()
    for e in events:
        if e['event']=='ids' and 0<=e['sim']-start<=600:
            false.update(set(e['ids'])-expected)
    with (run/'trajectory.csv').open() as f:
        for r in csv.DictReader(f):
            if 0<=float(r['elapsed_sim_s'])<=600:
                false.update({int(i) for i in r['registered_ids'].split()}-expected)
    return starts[0].get('ids',[]),sorted(false)


def adjudicate(run,world,policy,measure):
    checked_files(run,world)
    if fingerprint(read(run/'policy_manifest.json'))!=policy['source_manifest_sha256']:
        raise ValueError('Recorded source manifest differs from enrollment')
    row=measure(run,world)
    metrics=read(run/'metrics.json');audit=read(run/'audit.json')
    expected={t['id'] for t in read(run/'scene.json')['targets']}
    prestart,after=after_start_ids(run,expected)
    false_final=set(metrics['final_ids'])-expected
    if 0<=metrics['elapsed_sim_s']<=600:after=sorted(set(after)|false_final)
    t,actual=start_pose(run)
    xy,yaw=errors(actual,world['start'])
    movie=run/'mission.mp4'
    if not movie.is_file() or movie.stat().st_size<=0 or sha(movie)!=audit['video']['sha256']:
        raise ValueError('Missing or changed audited full camera video')
    if metrics['elapsed_wall_s']<0 or not math.isfinite(metrics['elapsed_wall_s']):
        raise ValueError('Invalid recorded wall-clock duration')
    issues=[]
    if not row['valid']:issues.append(row.get('invalid_reason') or 'native validation failed')
    if xy>XY_TOL or yaw>YAW_TOL:issues.append('measured world start differs from enrolled pose')
    if prestart:issues.append('IDs already present at start')
    row.update(valid=not issues,invalid_reason='; '.join(issues) or None,
               strict=bool(row['strict'] and not issues and not after),
               start_pose_sample_s=t,start_world_xyyaw=actual,start_config_xyyaw=world['start'],
               start_xy_error_m=xy,start_yaw_error_rad=yaw,prestart_ids=prestart,
               false_registered_ids_after_start=after,unexpected_final_ids=sorted(false_final),expected_ids=sorted(expected),
               episode_elapsed_wall_s=metrics['elapsed_wall_s'],video_sha256=audit['video']['sha256'],
               video=str(movie),source_manifest_sha256=policy['source_manifest_sha256'],
               independent_within_1p5_after_registration=sorted(map(int,row['after_registration_near_s'])),
               board_run=run.name,
               evidence_sha256={**{f:sha(run/f) for f in ('policy_manifest.json','metrics.json','audit.json',
                                   'events.jsonl','trajectory.csv','headings.csv')},'mission.mp4':audit['video']['sha256']})
    return row


def summarize(protocol,revision,entries):
    tasks,worlds=plan(protocol,revision)
    rows={(r['pair'],r['role']):r for r in entries}
    allowed={(t['pair'],role) for t in tasks for role in t['order']}
    if len(rows)!=len(entries) or not set(rows)<=allowed:raise ValueError('Duplicate or foreign episode')
    pairs=[]
    for t in tasks:
        a,b=(rows.get((t['pair'],role)) for role in ('baseline','candidate'))
        if not a or not b:continue
        item=dict(pair=t['pair'],seed=t['seed'],repeat=t['repeat'],valid=False,
                  baseline_strict=bool(a.get('strict')),candidate_strict=bool(b.get('strict')))
        if a.get('valid') and b.get('valid'):
            xy,yaw=errors(a['start_world_xyyaw'],b['start_world_xyyaw'])
            item.update(start_distance_m=xy,start_yaw_difference_rad=yaw,
                        valid=bool(xy<=XY_TOL and yaw<=YAW_TOL and
                                   a['world_sha256']==b['world_sha256']==worlds[t['seed']]['files']['world/osp_studio.sdf']
                                   and a['expected_ids']==b['expected_ids']))
            if item['valid']:
                item['joint_success_sim_time_delta_s']=(b['elapsed_s']-a['elapsed_s']
                                                      if a['strict'] and b['strict'] else None)
        pairs.append(item)
    complete=len(pairs)==len(tasks) and all(p['valid'] for p in pairs)
    return dict(revision=revision,recorded_episodes=len(rows),planned_episodes=2*len(tasks),
                completed_pairs=len(pairs),valid_pairs=sum(p['valid'] for p in pairs),
                distinct_completed_layouts=len({p['seed'] for p in pairs if p['valid']}),
                distinct_planned_layouts=len({t['seed'] for t in tasks}),
                valid_anchor_repetitions=sum(p['valid'] and p['seed']==ANCHOR for p in pairs),
                baseline_strict_successes=sum(p['baseline_strict'] for p in pairs if p['valid']),
                candidate_strict_successes=sum(p['candidate_strict'] for p in pairs if p['valid']),
                complete_valid=complete,conclusion='full paired evidence available for review' if complete else 'undetermined',
                pairs=pairs)


def native_tools():
    sys.path.insert(0,str(ENGINEERING));sys.path.insert(0,str(ENGINEERING/'validation'))
    import run
    from validate import identity,measure
    from board_audit import reconcile
    return run,identity,measure,reconcile


def ledger(session,protocol,revision):
    meta=session/'enrollment.json'
    enrolled=dict(protocol_sha256=fingerprint(protocol),revision=revision,
                  baseline_source=protocol['policies']['baseline']['source_manifest_sha256'],
                  candidate_source=protocol['policies'][revision]['source_manifest_sha256'])
    if meta.exists():
        if read(meta)!=enrolled:raise ValueError('Cannot reuse another protocol or policy session')
    else:
        if any(session.iterdir()):raise ValueError('Nonempty session without enrollment')
        save(meta,enrolled)
    p=session/'episodes.json'
    return p,read(p) if p.exists() else []


def native_run(root,revision,anchor_only=False):
    protocol=read(root/'protocol.json')
    tasks,worlds=plan(protocol,revision,anchor_only)
    runner,identity,measure,reconcile=native_tools()
    records=runner.HOME/'corridor-board-73bbb05/recordings'
    cards=Path(os.environ['CAMERA_BENCH_CARDS'])
    with ExitStack() as stack:
        for name in ('.domain.lock','.second-domain.lock'):
            fd=stack.enter_context((runner.HOME/'track-recordings'/name).open('a'))
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        runner.guard()
        # Native full-video finalization uses the demo-track renderer for exam fixtures.
        previously_heldout=runner.HELDOUT
        stack.callback(setattr,runner,'HELDOUT',previously_heldout)
        runner.HELDOUT=()
        session=root/revision;session.mkdir(exist_ok=True)
        ledger_path,entries=ledger(session,protocol,revision)
        summarize(protocol,revision,entries)
        for t in tasks:
            seed=t['seed'];world=worlds[seed]
            suffix=tag(protocol,revision,t['pair'])
            stage=session/f'pair-{suffix}'
            for role in t['order']:
                name=f'v151-survey-{role}-track-m{seed}-{suffix}'
                run=records/name
                policy=protocol['policies']['baseline' if role=='baseline' else revision]
                old=next((r for r in entries if r['pair']==t['pair'] and r['role']==role),None)
                if read(root/'protocol.json')!=protocol:raise RuntimeError('Protocol changed during session')
                verify_inputs(root,protocol,revision,world,identity)
                if old:
                    if old['run']!=str(run) or old['seed']!=seed:raise ValueError('Saved native run mismatch')
                    if old.get('state')!='evaluated':
                        raise RuntimeError('Incomplete attempt is preserved: '+str(run))
                    for filename,expected in old['evidence_sha256'].items():
                        if sha(run/filename)!=expected:raise ValueError('Saved evidence changed: '+str(run/filename))
                    checked_files(run,world)
                    continue
                runner.guard()
                if shutil.disk_usage(root).free<3*1024**3:
                    raise RuntimeError('Insufficient storage for next full-video trial')
                stage.mkdir(exist_ok=True)
                fixture_link=stage/'fixtures'
                if not fixture_link.exists():fixture_link.symlink_to(root/'fixtures',target_is_directory=True)
                if fixture_link.resolve()!=(root/'fixtures').resolve():raise ValueError('Unexpected fixture link')
                shim=stage/'protocol.json';digest_shim=dict(worlds={str(seed):world['files']['world/osp_studio.sdf']})
                if not shim.exists():save(shim,digest_shim)
                if read(shim)!=digest_shim:raise ValueError('Native fixture handoff differs')
                claim=stage/f'{role}.attempt.json'
                if claim.exists() or run.exists() or (runner.HOME/'track-recordings'/name).is_symlink() or (cards/name).exists():
                    raise FileExistsError('A native attempt already exists; preserved without retry: '+name)
                attempt=dict(pair=t['pair'],role=role,subject='baseline' if role=='baseline' else revision,
                             seed=seed,repeat=t['repeat'],run=str(run),began_utc=utc(),state='started')
                with claim.open('x') as f:json.dump(attempt,f)
                try:
                    actual_run=runner.trial(stage,role,seed,Path(policy['workspace']))
                    if actual_run!=run:raise ValueError('Unexpected native run path')
                    if read(root/'protocol.json')!=protocol:raise RuntimeError('Protocol changed during episode')
                    verify_inputs(root,protocol,revision,world,identity)
                    report=adjudicate(run,world,policy,measure)
                except BaseException as exc:
                    entries.append(dict(**{**attempt,'state':'incomplete'},ended_utc=utc(),valid=False,strict=False,why=repr(exc)))
                    save(ledger_path,entries);save(session/'result.json',summarize(protocol,revision,entries))
                    raise
                row={**attempt,**report,'state':'evaluated','ended_utc':utc()}
                entries.append(row);save(ledger_path,entries)
                try:row['board_status']=reconcile(run,cards,BOARD)
                except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:row['board_status']=str(exc)
                save(ledger_path,entries);save(session/'result.json',summarize(protocol,revision,entries))
                print('DONE',revision,t['pair'],role,seed,row['strict'],row['board_status'],flush=True)
        summary=summarize(protocol,revision,entries)
        save(session/'result.json',summary)
        return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('run','status'));p.add_argument('root',type=Path)
    p.add_argument('--revision',choices=('R3','R4'),required=True);p.add_argument('--anchor-only',action='store_true')
    args=p.parse_args();root=args.root.resolve()
    if args.action=='run':summary=native_run(root,args.revision,args.anchor_only)
    else:
        protocol=read(root/'protocol.json');p=root/args.revision/'episodes.json'
        if p.exists():
            enrolled=read(p.parent/'enrollment.json')
            if enrolled['protocol_sha256']!=fingerprint(protocol) or enrolled['revision']!=args.revision:
                raise ValueError('Status ledger belongs to another protocol')
        summary=summarize(protocol,args.revision,read(p) if p.exists() else [])
    print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
