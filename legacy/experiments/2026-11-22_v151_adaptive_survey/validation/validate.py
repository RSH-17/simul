"""Prepare, inspect or explicitly run a locked, simulation-only blind comparison."""
import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import sys

from blind_fixtures import FAMILIES, build, digest
from decisions import screen, pairs_report

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from compare import report as screen_report
import run as runner


def read(p):return json.loads(p.read_text())


def write(p,data):
    tmp=p.with_suffix('.new')
    tmp.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(p)


def checksum(data):
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def identity(ws):
    src=ws/'src/osp_mission'
    if not (ws/'install/setup.bash').exists() or not src.is_dir():raise ValueError(f'Unbuilt workspace {ws}')
    files={str(p.relative_to(src)):digest(p) for p in sorted(src.rglob('*'))
           if p.is_file() and p.suffix in ('.py','.yaml','.json') and '__pycache__' not in p.parts}
    configs=[Path.home()/'simul/policy_releases/2026-11-13_face_intent_v2/policy.json',
             Path.home()/'simul/robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml']
    helpers=[runner.HERE/'trial.py',runner.EXPERIMENTS/'2026-11-21_camera_aware_policy/camera.launch.py',
             Path(os.environ['CAMERA_BENCH_HELPERS'])/'2026-11-19_pair_7_103/measure.py']
    # The symlink installation imports source through its egg link.
    egg=list((ws/'install/osp_mission/lib').glob('python*/site-packages/osp-mission.egg-link'))
    if egg:
        effective=Path(egg[0].read_text().splitlines()[0])/'osp_mission'
    else:
        locations=list((ws/'install/osp_mission/lib').glob('python*/site-packages/osp_mission'))
        if len(locations)!=1:raise ValueError('Missing installed Python package '+str(ws))
        effective=locations[0]
    if not (effective/'mission.py').exists():raise ValueError('Missing installed entrypoint '+str(ws))
    package={f.name:digest(f) for f in effective.glob('*.py')}
    bin_root=ws/'install/osp_mission/lib/osp_mission'
    return dict(workspace=str(ws), source_manifest_sha256=checksum(files),
                installed_python=package,installed_executables={p.name:digest(p) for p in sorted(bin_root.glob('*')) if p.is_file()},
                launch_configs={p.name:digest(p) for p in configs+helpers})


def assert_inputs(root, manifest):
    for item in manifest['worlds']:
        folder=root/'fixtures'/f"map-{item['seed']}"
        for rel,sha in item['files'].items():
            if digest(folder/rel)!=sha:raise ValueError('Changed blind fixture '+str(folder/rel))
    for meta in manifest['policies'].values():
        if meta!=identity(Path(meta['workspace'])):raise ValueError('Changed frozen policy or runtime inputs')
    for name,sha in manifest['validator'].items():
        if digest(HERE/name)!=sha:raise ValueError('Changed validation code '+name)


def prepare(args):
    root=args.root.resolve()
    if (root/'manifest.json').exists():
        m=read(root/'manifest.json')
        if str(args.candidate.resolve())!=m['policies']['candidate']['workspace']:raise ValueError('Different enrolled candidate')
        assert_inputs(root,m);print('Previously frozen suite',checksum(m));return
    root.mkdir(parents=True,exist_ok=True);fixtures=root/'fixtures';fixtures.mkdir(exist_ok=True)
    policies={role:identity(path.resolve()) for role,path in (('baseline',args.baseline),('candidate',args.candidate))}
    # Draw only after source digests are fixed. Identical policy/topology never
    # redraws a seed in this enrollment. Geometry rejection uses no performance.
    previous=read(root/'enrollment.json') if (root/'enrollment.json').exists() else None
    if previous and previous['policies']!=policies:raise ValueError('Different enrolled source, cannot continue')
    nonce=previous['nonce'] if previous else secrets.token_hex(24)
    seed_for=lambda i: int(hashlib.sha256((nonce+str(i)).encode()).hexdigest()[:12],16)%800000000+100000000
    enrolled=dict(policies=policies,nonce=nonce,seeds=[seed_for(i) for i in range(6)])
    write(root/'enrollment.json',enrolled)
    records=Path.home()/'corridor-board-73bbb05/recordings'
    prior={p.name for p in records.iterdir()}
    if len(set(enrolled['seeds']))!=6 or any(any(f'-m{seed}-' in n for n in prior) for seed in enrolled['seeds']):
        raise ValueError('Seed already exercised; enrollment invalid, do not redraw')
    worlds=[build(fixtures,seed,family) for seed,family in zip(enrolled['seeds'],FAMILIES)]
    sequence=list(range(6))+[1,2,4]
    pairs=[]
    for i,index in enumerate(sequence):
        w=worlds[index];repeat=int(i>=6)
        # Complement the first exposure order on repeated geometries.
        candidate_first=bool((index%2)^repeat)
        order=['candidate','baseline'] if candidate_first else ['baseline','candidate']
        pairs.append(dict(pair=i+1,repeat=repeat,seed=w['seed'],topology=w['topology'],order=order))
    manifest=dict(**enrolled,worlds=worlds,pairs=pairs,source_screen=str(args.screen.resolve()),
                  camera=[480,480,5],max_mission_sim_s=600,max_episode_wall_s=2400,
                  validator={p.name:digest(p) for p in (HERE/'blind_fixtures.py',HERE/'decisions.py',HERE/'validate.py')})
    write(root/'manifest.json',manifest);print('Frozen suite',checksum(manifest),'worlds',len(worlds))


def raw_witnesses(run,registered,targets):
    witnessed={};false=set();counts=0;settled=None
    with (run/'trajectory.csv').open() as f:
        for row in csv.DictReader(f):
            t=float(row['elapsed_sim_s'])
            if not 0<=t<=600:continue
            ids=set(map(int,row['registered_ids'].split()));false.update(ids-targets.keys())
            xy=(float(row['world_x']),float(row['world_y']))
            if settled is None:settled=xy
            counts+=1
            for i in ids & targets.keys() & registered.keys():
                if t+1e-8>=registered[i] and math.dist(xy,targets[i])<=1.5:witnessed.setdefault(i,t)
    return witnessed,false,counts,settled


def measure(run,world):
    m=read(run/'metrics.json');audit=read(run/'audit.json');scene=read(run/'scene.json')
    targets={t['id']:(t['x'],t['y']) for t in scene['targets']};expected=set(targets)
    walls={t['id'] for t in scene['targets'] if t['attached_to'].startswith('wall')}
    registered={int(i):t for i,t in m['registered_s'].items() if t<=600}
    near,false,n,start=raw_witnesses(run,registered,targets)
    false|=set(m['final_ids'])-expected
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    started=next(x for x in events if x['event']=='start_ack')
    for e in events:
        if e['event']=='ids' and 0<=e['sim']-started['sim']<=600:false.update(set(e['ids'])-expected)
    errors={}
    alignment=m.get('world_from_map_initial_alignment')
    if alignment:
        rot,offset=alignment['rotation'],alignment['translation']
        for ident,xy in audit['saved_map_xy'].items():
            i=int(ident)
            if i in expected:
                mapped=tuple(sum(v*x for v,x in zip(r,xy))+off for r,off in zip(rot,offset))
                errors[i]=math.dist(mapped,targets[i])
    camera=m['camera'];info=camera.get('camera_info') or {}
    valid=bool(n and m['hazard_packets'] and info.get('width')==info.get('height')==480
               and camera['requested_fps']==5 and m['world_sha256']==world['files']['world/osp_studio.sdf']
               and not started.get('ids') and start is not None)
    strict=bool(valid and set(m['final_ids'])==expected==set(registered)==set(near)==set(errors)
                and max(errors.values(),default=math.inf)<=.25 and m['policy_sensor_proof_complete']
                and m['bumper_events']==0 and not false and m['elapsed_sim_s']<=600.)
    return dict(valid=valid,invalid_reason=None if valid else 'missing independent evidence, pre-start IDs, world or camera mismatch',
                strict=strict,elapsed_s=m['elapsed_sim_s'],settled_xy=start,world_sha256=m['world_sha256'],
                registered_ids=sorted(registered),after_registration_near_s=near,unexpected_ids=sorted(false),
                wall_registered=len(walls & registered.keys()),box_registered=len((expected-walls)&registered.keys()),
                wall_near=len(walls & near.keys()),box_near=len((expected-walls)&near.keys()),
                expected_wall=len(walls),expected_box=len(expected-walls),map_errors_m=errors,
                sensor_proof=m['policy_sensor_proof_complete'],bumps=m['bumper_events'],distance_m=m['path_length_m'],
                stopped_s=m['stationary_time_s'],terminal_reason=m['reason'],truth_samples=n,
                min_scan_m=m.get('minimum_valid_scan_range_m'),camera_info=info)


def screen_frozen(manifest):
    root=Path(manifest['source_screen'])
    result=screen(screen_report(root))
    for role,expected in manifest['policies'].items():
        for run in read(root/role/'runs.json') if (root/role/'runs.json').exists() else []:
            if checksum(read(Path(run)/'policy_manifest.json'))!=expected['source_manifest_sha256']:
                result['eligible']=False;result['reasons'].append(role+': screen used another source revision');break
    return result


def execute(args):
    root=args.root.resolve();manifest=read(root/'manifest.json');assert_inputs(root,manifest)
    gate=screen_frozen(manifest)
    if not gate['eligible']:
        print(json.dumps(gate));return
    if args.action=='gate':print(json.dumps(gate));return
    locks=[(Path.home()/'track-recordings'/n).open('a') for n in ('.domain.lock','.second-domain.lock')]
    for fd in locks:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    runner.guard()
    # In this separate process HELDOUT only controls the recording name. The
    # legacy '-track-' video finalizer loads the demo room's south recess.
    runner.HELDOUT = tuple(w['seed'] for w in manifest['worlds'])
    dest=root/'episodes.json';rows=read(dest) if dest.exists() else []
    for task in manifest['pairs']:
        world=next(w for w in manifest['worlds'] if w['seed']==task['seed'])
        for role in task['order']:
            key=task['pair'],role
            if any((r['pair'],r['role'])==key for r in rows):continue
            assert_inputs(root,manifest);runner.guard()
            if shutil.disk_usage(root).free<3*1024**3:
                raise RuntimeError('Under 3 GiB free: gate paused before next launch, no files removed')
            tag=str(int(manifest['nonce'][:10],16))+f"{task['pair']:02d}"
            stage=root/f'pair-{tag}';stage.mkdir(exist_ok=True)
            if not (stage/'fixtures').exists():(stage/'fixtures').symlink_to(root/'fixtures',target_is_directory=True)
            if not (stage/'protocol.json').exists():write(stage/'protocol.json',dict(worlds={str(world['seed']):world['files']['world/osp_studio.sdf']}))
            run=Path.home()/'corridor-board-73bbb05/recordings'/f"v151-survey-{role}-heldout-m{world['seed']}-{tag}"
            try:
                if run.exists():
                    raise RuntimeError('Prior recording exists without an adjudicated outcome; avoid a selective rerun')
                path=runner.trial(stage,role,world['seed'],Path(manifest['policies'][role]['workspace']))
                if path!=run:raise RuntimeError('Unexpected native result path')
                measurement=measure(path,world)
            except (Exception,KeyboardInterrupt) as exc:
                invalid=dict(valid=False,strict=False,settled_xy=None,world_sha256=world['files']['world/osp_studio.sdf'],
                             invalid_reason=repr(exc),elapsed_s=None)
                rows.append(dict(pair=key[0],role=role,run=run.name,**invalid));write(dest,rows)
                write(root/'result.json',pairs_report(manifest['pairs'],rows))
                raise
            rows.append(dict(pair=key[0],role=role,run=run.name,**measurement));write(dest,rows)
            write(root/'result.json',pairs_report(manifest['pairs'],rows))
            print('Completed pair',task['pair'],role,measurement['strict'],flush=True)
    print('Complete',root/'result.json')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    cmd=parser.add_subparsers(required=True,dest='action')
    a=cmd.add_parser('prepare');a.add_argument('root',type=Path);a.add_argument('--screen',type=Path,required=True)
    a.add_argument('--baseline',type=Path,required=True);a.add_argument('--candidate',type=Path,required=True)
    for name in ('gate','run','status'):
        a=cmd.add_parser(name);a.add_argument('root',type=Path)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args)
    elif args.action=='status':
        m=read(args.root/'manifest.json');f=args.root/'episodes.json'
        print(json.dumps(pairs_report(m['pairs'],read(f) if f.exists() else []),indent=2))
    else:execute(args)


if __name__=='__main__':main()
