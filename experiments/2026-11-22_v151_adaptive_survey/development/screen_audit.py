"""Rescore completed development recordings. This command never lists or runs blind trials."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'validation'))
from decisions import screen
from validate import checksum, identity, measure

HOME = Path.home()
RECORDINGS = HOME / 'corridor-board-73bbb05/recordings'
ARCHIVE = HOME / 'v151-track-five-boxes5-1795247134501974510'
R2 = HOME / 'v151-adaptive-survey-paired-1122'
ROOTS = {'R2': R2, 'R3': HOME / 'v151-object-followthrough-screen-112203'}
REGRESSION = {112601, 112602, 112603, 112604, 112605}
HELD = {112801, 112802}
WORKSPACES = {'baseline': 'flow_v151_exposure_sync_r2_ws', 'R2': 'v151_adaptive_survey1122_r2_ws',
              'R3': 'v151_object_followthrough1122_r3_ws'}
# Current execution identities measured independently. The R2 and baseline digests
# also appear in the published enrollment receipt. R3 is a development-stage snapshot.
FINGERPRINTS = {'baseline': '82410a7721c0b6dca32e95e5e9d43c30537360dd29a8f579ecbc0d53bd31317a',
                'R2': 'ef4fa250976927630188bcd816b007e4489366439104c30c4eda72be077a1365',
                'R3': '06513b947120dc1fe0c99f83a0b89ff5422db1dd57e7a13565a9e7a3e23d3b8a'}


def read(path):
    return json.loads(path.read_text())


def run_path(group, revision, seed, name):
    if group == 'archive':
        expected = seed in REGRESSION and name.startswith(f'v151-boxes5-track-m{seed}-trial') and name.rsplit('trial', 1)[-1].isdigit()
    else:
        token = '1122' if group == 'baseline' or revision == 'R2' else '112203'
        expected = seed in (HELD if group == 'baseline' else REGRESSION | HELD)
        split = 'heldout' if seed in HELD else 'track'
        expected = expected and name == f'v151-survey-{group}-{split}-m{seed}-{token}'
    path = RECORDINGS / name
    if not expected or path.resolve() != path or path.parent != RECORDINGS:
        raise ValueError('Unexpected development recording name')
    # No fixture enumeration. Only the original judge receives the target record
    # already associated with this specific finished, allowlisted engineering run.
    allowed_origins = (ARCHIVE / f'map-{seed}', R2 / 'fixtures' / f'map-{seed}', path)
    if (path / 'scene.json').resolve() not in [p / 'scene.json' for p in allowed_origins]:
        raise ValueError('Unrecognized source of completed development recording')
    return path


def audit_revision(revision, first_two=False):
    root = ROOTS[revision]
    protocol = read(root / 'protocol.json')
    if set(protocol['regression']) != REGRESSION or set(protocol['heldout']) != HELD:
        raise ValueError('Unexpected development protocol')
    issues, rows, current = [], [], {}
    for role in ('baseline', revision):
        signature = identity(HOME / WORKSPACES[role])
        now = checksum(signature)
        current[role] = {'runtime_sha256': now, 'source_sha256': signature['source_manifest_sha256'],
                         'reference_match': now == FINGERPRINTS[role]}
        if now != FINGERPRINTS[role]:
            issues.append(f'{role}: changed live source, installation or runtime fingerprint')
    for group, directory in [('archive', ARCHIVE), ('baseline', root / 'baseline'), ('candidate', root / 'candidate')]:
        if not (directory / 'report.json').exists():
            continue
        index = set(read(directory / 'runs.json'))
        seen = set()
        for record in read(directory / 'report.json')['runs']:
            seed, name = record['seed'], record['run']
            if first_two and group == 'candidate' and seed not in (112601, 112605):
                continue
            if seed in seen:
                raise ValueError(f'Duplicate {group} seed {seed}')
            seen.add(seed)
            path = run_path(group, revision, seed, name)
            if str(path) not in index:
                raise ValueError('Report row absent from completed recording list')
            m = read(path / 'metrics.json')
            expected = protocol['worlds'][str(seed)]
            raw = measure(path, {'files': {'world/osp_studio.sdf': expected}})
            old_near = {int(i): t for i,t in m['registered_and_within_s'].items()}
            reg = {int(i): t for i,t in m['registered_s'].items()}
            new_near = raw['after_registration_near_s']
            src = path / 'policy_manifest.json'
            sha = checksum(read(src)) if src.exists() else None
            role = 'baseline' if group != 'candidate' else revision
            if sha != current[role]['source_sha256'] and (sha is not None or group != 'archive'):
                issues.append(f'{name}: missing or changed per-run policy source')
            if not raw['valid'] or m['world_sha256'] != record['world_sha256']:
                issues.append(f'{name}: invalid evidence or changed world')
            if not math.dist(raw['settled_xy'], record['start_world_pose'][:2]) <= .15:
                issues.append(f'{name}: different recorded and reconstructed start')
            original_false = sorted(set(record['registered_ids']) - {7, 32, 103})
            # Preserve the precommitted live-map strict requirement even where
            # the independent definition checks only the persisted map.
            conservative_strict = bool(record['strict_saved_map_success'] and raw['strict'])
            rows.append(dict(run=name,seed=seed,group=group,valid=raw['valid'],
                original_strict=record['strict_saved_map_success'],raw_strict=raw['strict'],
                conservative_strict=conservative_strict,original_near_ids=sorted(old_near),raw_near_ids=sorted(new_near),
                originally_unexpected=original_false,raw_unexpected=raw['unexpected_ids'],
                registered=raw['registered_ids'],wall_reg=raw['wall_registered'],box_reg=raw['box_registered'],
                wall_near=raw['wall_near'],box_near=raw['box_near'],bumps=raw['bumps'],
                early_witness=[dict(id=i,old_s=round(old_near[i],6),registered_s=round(reg[i],6),
                                    later_valid_s=new_near.get(i))
                               for i in sorted(old_near.keys() & reg.keys()) if old_near[i]+1e-8 < reg[i]],
                source_sha256=sha,world_sha256=m['world_sha256'],settled_xy=raw['settled_xy']))
    idx={(('candidate' if r['group']=='candidate' else 'baseline'),r['seed']):r for r in rows}
    # Use the completed engineering comparison for configured-start / host pairing;
    # the independent raw reconstruction supplies world/start/ID evidence.
    final_export=HOME/'v151-adaptive-final-export1122/comparison.json'
    snapshot=final_export if revision=='R2' and not first_two and final_export.is_file() else root/'paired_comparison.json'
    paired=read(snapshot) if snapshot.exists() else {}
    filtered={'complete':False}
    for split,seeds in [('regression',REGRESSION),('heldout',HELD)]:
        verified=[]
        for original in paired.get(split,{}).get('pairs',[]):
            s=original['seed']
            if s not in seeds or not all((role,s) in idx for role in ('candidate','baseline')):
                continue
            pair=copy.deepcopy(original)
            a,b=(idx[role,s] for role in ('baseline','candidate'))
            distance=math.dist(a['settled_xy'],b['settled_xy'])
            if a['world_sha256']!=b['world_sha256'] or distance>.15:
                issues.append(f'{s}: different world or settled start')
            pair['settled_start_distance_m']=distance
            for role,raw in [('baseline',a),('candidate',b)]:
                old=pair[role]
                if (old['run'] != raw['run'] or old['sha256']!=raw['world_sha256']
                        or old['strict']!=raw['original_strict'] or sorted(old['verified_near_ids'])!=raw['original_near_ids']):
                    issues.append(f'{raw["run"]}: original summary diverges from episode record')
                ids=set(raw['registered']) & set(old['final_ids'])
                near=set(raw['raw_near_ids']) & set(raw['original_near_ids'])
                for prefix in ('wall','box'):
                    hosts=set(old[prefix+'_ids'])
                    old[prefix+'_reg']=len(ids & hosts)
                    old[prefix+'_near']=len(near & hosts)
                old['bumps']=raw['bumps']
                old['strict']=raw['conservative_strict']
                old['unexpected_ids']=raw['raw_unexpected']
            verified.append(pair)
        filtered[split]={'pairs':verified}
    filtered['complete']=all(len(filtered[k]['pairs'])==n for k,n in [('regression',5),('heldout',2)])
    original_gate=screen(paired) if paired else None
    conservative_gate=screen(filtered)
    different=(filtered['complete'] and original_gate is not None and original_gate['eligible']!=conservative_gate['eligible'])
    summary=dict(runs=len(rows),strict_before=sum(r['original_strict'] for r in rows),
                 strict_raw=sum(r['raw_strict'] for r in rows),strict_conservative=sum(r['conservative_strict'] for r in rows),
                 near_before=sum(len(r['original_near_ids']) for r in rows),near_after=sum(len(r['raw_near_ids']) for r in rows),
                 raw_valid=sum(r['valid'] for r in rows),unexpected_before=sum(len(r['originally_unexpected']) for r in rows),
                 unexpected_after=sum(len(r['raw_unexpected']) for r in rows),
                 early_witnesses=sum(len(r['early_witness']) for r in rows),
                 strict_changes=sum(r['conservative_strict']!=r['original_strict'] for r in rows),
                 approach_set_changes=sum(r['original_near_ids']!=r['raw_near_ids'] for r in rows),
                 archived_without_per_run_source=sum(r['group']=='archive' and r['source_sha256'] is None for r in rows))
    return dict(revision=revision,scope='completed engineering development recordings only',summary=summary,
                complete=filtered['complete'],source=current,rows=rows,integrity_issues=issues,
                original_gate=original_gate,conservative_gate=conservative_gate,gate_reversal=different,
                allow_independent_dispatch=bool(filtered['complete'] and not issues and not different and
                    original_gate and original_gate['eligible'] and conservative_gate['eligible']),
                limits=['Archived five have contemporaneous release evidence but no per-run executable digest.',
                        'Geometry based on recorded world SHA, paired launch metadata and start XY. No raw world/SDF/mesh inspection.',
                        'Same native observer and truth input. Re-adjudication is independent of its cached distance decision.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('revision', choices=ROOTS)
    parser.add_argument('--snapshot-first-two',action='store_true')
    parser.add_argument('--require-eligible',action='store_true')
    args=parser.parse_args()
    if args.snapshot_first_two and args.require_eligible:
        parser.error('A partial snapshot cannot qualify')
    data=audit_revision(args.revision,args.snapshot_first_two)
    print(json.dumps(data,indent=2,sort_keys=True,allow_nan=False))
    return int(args.require_eligible and not data['allow_independent_dispatch'])


if __name__=='__main__':
    sys.exit(main())
