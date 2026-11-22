"""Native 8780 randomized exam: enroll before driving, replay the same world for each policy."""
import argparse
from collections import Counter
from contextlib import ExitStack
import fcntl
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import sys
import time

import numpy as np

from fixture import VIEW_SOURCES, build, digest, read, save
from scoring import score

HERE=Path(__file__).resolve().parent
PRIOR=HERE.parent/'2026-11-22_exam_track_validation'
ENGINE=HERE.parent/'2026-11-22_v151_adaptive_survey'
sys.path[:0]=[str(PRIOR),str(ENGINE/'validation'),str(ENGINE)]
import paired
from validate import identity, measure
import run as runner
from board_audit import reconcile


def file_identity():
    active=[HERE/f for f in ('geometry.py','fixture.py','scoring.py','campaign.py','trial.py','exam_camera.launch.py','run.sh')]
    active.extend(VIEW_SOURCES)
    active += [PRIOR/'paired.py',ENGINE/'run.py',ENGINE/'validation/validate.py',
               HERE.parent/'2026-11-21_camera_aware_policy/strict_trial.py',
               HERE.parent/'2026-11-20_v12_camera_rate/trial.py',
               Path(os.environ['CAMERA_BENCH_HELPERS'])/'2026-11-19_corridor_box/finish_trial.py']
    return {str(p):digest(p) for p in active}


def configure(root,candidate,label,known_count,tables):
    if known_count not in (0,3):raise ValueError('Use the publicly supplied exam count of three or undisclosed count')
    root.mkdir(parents=True,exist_ok=True)
    fixtures=root/'fixtures';fixtures.mkdir(exist_ok=True)
    base=identity(runner.BASELINE); subject=identity(candidate)
    if subject['source_manifest_sha256']==base['source_manifest_sha256']:raise ValueError('Policies are identical')
    common=dict(baseline=base,candidate=subject,candidate_label=label,known_marker_count=known_count,
                furniture=tables,checksums=file_identity(),camera=[480,480,5],duration_s=600,
                limits=dict(obstacles=[5,5],obstacle_geometry='original_exam_boxes',unknown_marker_count=[1,5],print_sizes_m=[.2,.25,.3]),
                survey_distant_hints=False,
                fixed_room='uFjtqfyJ',physical_drive=False,domain=174)
    enr=root/'registration.json'
    if enr.exists():
        old=read(enr)
        if any(old.get(k)!=v for k,v in common.items()):raise ValueError('Policy, generator or requested exam rules changed')
    else:
        nonce=secrets.token_hex(32)
        # Five independent layouts and one prespecified repeat of the first.
        draws=[int(hashlib.sha256(f'{nonce}/world/{i}'.encode()).hexdigest()[:8],16) for i in range(5)]
        if len(set(draws))!=5:raise ValueError('Duplicate random draw')
        records=runner.HOME/'corridor-board-73bbb05/recordings'
        if any(any(f'-m{s}-' in p.name for s in draws) for p in records.iterdir()):raise ValueError('A sampled seed has already been used')
        old=dict(**common,nonce=nonce,draws=draws,repeated_index=0)
        save(enr,old)
    proto_file=root/'protocol.json'
    if proto_file.exists():
        doc=read(proto_file)
        if doc['registration_sha256']!=digest(enr):raise ValueError('Registration changed')
        for world in doc['worlds']:paired.checked_files(fixtures/f"map-{world['seed']}",world)
        return doc
    original=runner.HOME/'track-recordings/.layouts/uFjtqfyJ'
    worlds=[]
    for seed in old['draws']:
        folder=fixtures/f'map-{seed}'; meta=root/f'geometry-{seed}.json'
        if meta.exists():
            w=read(meta);paired.checked_files(folder,w)
        else:
            # Number draws, when the number is unknown to the controller, never
            # enter a policy parameter. They belong to the fixture builder only.
            rng=np.random.default_rng([seed,0x544234])
            number=known_count or int(rng.integers(1,6))
            mode=tables if tables!='both' else ('fixed' if len(worlds)%2==0 else 'shuffle')
            w=build(original,folder,seed,number,mode)
            save(meta,w)
        worlds.append(w)
    indices=[0,1,2,3,4,0]
    tasks=[dict(pair=i+1,world=index,repeat=int(i==5),
                order=['candidate','baseline'] if i%2==0 else ['baseline','candidate'])
           for i,index in enumerate(indices)]
    result=dict(registration_sha256=digest(enr),worlds=worlds,plan=tasks)
    save(proto_file,result)
    return result


def totals(records):
    rows=[r['score'] for r in records]
    elapsed=sum(r['goal_complete_by_s'] if r['strict'] else 600. for r in rows)
    return dict(runs=len(rows),strict=sum(r['strict'] for r in rows),
                registered=sum(r['registered'] for r in rows),all_markers=sum(r['expected'] for r in rows),
                verified=sum(r['verified'] for r in rows),wall_verified=sum(r['wall_near'] for r in rows),
                walls=sum(r['wall'] for r in rows),obstacle_verified=sum(r['obstacle_near'] for r in rows),
                obstacles=sum(r['obstacle'] for r in rows),bumps=sum(r['bumps'] for r in rows),
                false_registered_instances=sum(len(r['false_ids']) for r in rows),
                mean_time_with_failures_at_limit_s=elapsed/len(rows) if rows else None,
                mean_actual_mission_s=sum(r['elapsed_s'] for r in rows)/len(rows) if rows else None,
                mean_travel_m=sum(r['travel_m'] for r in rows)/len(rows) if rows else None)


def summary(root):
    protocol=read(root/'protocol.json'); ledger=root/'episodes.json'
    eps=read(ledger) if ledger.exists() else []
    groups={(r['pair'],r['role']):r for r in eps}
    if len(groups)!=len(eps):raise ValueError('Duplicate scored episode')
    pairs=[]
    for t in protocol['plan']:
        if not all((t['pair'],role) in groups for role in t['order']):continue
        b,c=(groups[t['pair'],role] for role in ('baseline','candidate'))
        w=protocol['worlds'][t['world']]
        xy,yaw=paired.errors(b['start'],c['start'])
        valid=bool(b['score']['valid'] and c['score']['valid'] and
                   b['world_sha256']==c['world_sha256']==w['files']['world/osp_studio.sdf'] and
                   xy<=paired.XY_TOL and yaw<=paired.YAW_TOL)
        pairs.append(dict(pair=t['pair'],seed=w['seed'],repeat=t['repeat'],valid=valid,
                          baseline=b,candidate=c))
    valid=[p for p in pairs if p['valid']]
    def summarize(rows):return {role:totals([p[role] for p in rows]) for role in ('baseline','candidate')}
    return dict(planned_pairs=len(protocol['plan']),finalized_episodes=len(eps),paired=len(pairs),valid_pairs=len(valid),
                distinct_scored_layouts=len({p['seed'] for p in valid}),first_runs=summarize([p for p in valid if not p['repeat']]),
                including_repeat=summarize(valid),pairs=pairs)


def run_all(root):
    reg=read(root/'registration.json');proto=read(root/'protocol.json')
    os.environ['OSP_PUBLIC_MARKER_COUNT']=str(reg['known_marker_count'])
    os.environ['OSP_SURVEY_DISTANT_HINTS']='true' if reg.get('survey_distant_hints') else 'false'
    ledger=root/'episodes.json'; status=root/'status.json'
    episodes=read(ledger) if ledger.exists() else []
    native=runner.HOME/'corridor-board-73bbb05/recordings'
    def check_inputs():
        if digest(root/'registration.json')!=proto['registration_sha256'] or read(root/'protocol.json')!=proto:
            raise ValueError('Exam inputs changed since registration')
        if reg['checksums']!=file_identity():raise ValueError('Active evaluation code changed')
        for r in ('baseline','candidate'):
            if identity(Path(reg[r]['workspace']))!=reg[r]:raise ValueError('Active policy changed: '+r)
    check_inputs()
    with ExitStack() as stack:
        for filename in ('.domain.lock','.second-domain.lock'):
            file=stack.enter_context((runner.HOME/'track-recordings'/filename).open('a'))
            save(status,dict(state='waiting_for_native_simulator',episodes=len(episodes)))
            fcntl.flock(file,fcntl.LOCK_EX)
        runner.guard();runner.HELDOUT=()
        for t in proto['plan']:
            w=proto['worlds'][t['world']];seed=w['seed']
            salt=f"{reg['nonce']}/episode/{t['pair']}"
            suffix=str(int(hashlib.sha256(salt.encode()).hexdigest()[:15],16))
            stage=root/('pair-'+suffix);stage.mkdir(exist_ok=True)
            ln=stage/'fixtures'
            if not ln.exists():ln.symlink_to(root/'fixtures',target_is_directory=True)
            if ln.resolve()!=(root/'fixtures').resolve():raise ValueError('Incorrect world directory')
            native_proto=stage/'protocol.json';shim=dict(worlds={str(seed):w['files']['world/osp_studio.sdf']})
            if not native_proto.exists():save(native_proto,shim)
            if read(native_proto)!=shim:raise ValueError('World key mismatch')
            for role in t['order']:
                check_inputs();paired.checked_files(root/'fixtures'/f'map-{seed}',w)
                name=f'v151-survey-{role}-track-m{seed}-{suffix}';run=native/name
                old=next((r for r in episodes if (r['pair'],r['role'])==(t['pair'],role)),None)
                if old:
                    if old['name']!=name or any(digest(run/key)!=sha for key,sha in old['evidence'].items()):
                        raise ValueError('Evidence from prior attempt changed')
                    continue
                claim=stage/f'{role}.attempt.json'
                if claim.exists() or run.exists():raise FileExistsError('Preserving existing unscored attempt: '+name)
                if shutil.disk_usage(root).free<3*1024**3:raise RuntimeError('Insufficient disk for a full camera episode')
                runner.guard()
                data=dict(pair=t['pair'],role=role,seed=seed,name=name)
                with claim.open('x') as f:json.dump(dict(**data,start_wall_unix=time.time()),f)
                save(status,dict(state='native_episode',**data,episodes=len(episodes)))
                native_here=runner.HERE
                try:
                    # All upstream simulation, navigation and recording stay in place.
                    # Only use this panel's passive observer in this Python process.
                    runner.HERE=HERE
                    actual=runner.trial(stage,role,seed,Path(reg[role]['workspace']))
                finally:runner.HERE=native_here
                if actual!=run:raise ValueError('Unexpected recording path')
                check_inputs()
                record=paired.adjudicate(run,w,reg[role],measure)
                result=score(record,read(run/'metrics.json'),read(run/'scene.json'))
                row=dict(**data,start=record['start_world_xyyaw'],world_sha256=record['world_sha256'],
                         source_sha256=record['source_manifest_sha256'],evidence=record['evidence_sha256'],score=result)
                episodes.append(row);save(ledger,episodes);save(root/'comparison.json',summary(root))
                try:reconcile(run,Path(os.environ['CAMERA_BENCH_CARDS']),paired.BOARD)
                except (KeyError,OSError,RuntimeError,ValueError) as ex:print('Dashboard refresh',name,str(ex),flush=True)
                print('DONE',t['pair'],role,'strict',result['strict'],'ID',result['registered'],
                      'approached',result['verified'],'of',result['expected'],flush=True)
        final=summary(root);save(root/'comparison.json',final)
        save(status,dict(state='complete' if final['valid_pairs']==len(proto['plan']) else 'invalid_pair',
                         episodes=len(episodes),valid_pairs=final['valid_pairs']))


def main():
    p=argparse.ArgumentParser(description=__doc__); cmd=p.add_subparsers(dest='act',required=True)
    setup=cmd.add_parser('prepare');setup.add_argument('root',type=Path);setup.add_argument('candidate',type=Path)
    setup.add_argument('--label',required=True);setup.add_argument('--known-count',type=int,choices=(0,3),required=True)
    setup.add_argument('--tables',choices=('fixed','shuffle','both'),required=True)
    for name in ('run','status'):cmd.add_parser(name).add_argument('root',type=Path)
    a=p.parse_args();root=a.root.resolve()
    if a.act=='prepare':
        root.mkdir(parents=True,exist_ok=True)
        with (root/'enroll.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            d=configure(root,a.candidate.resolve(),a.label,a.known_count,a.tables)
        print('Frozen random layouts',len(d['worlds']),'pairs',len(d['plan']),'protocol',digest(root/'protocol.json'))
    elif a.act=='run':
        with (root/'owner.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            try:run_all(root)
            except Exception as e:
                save(root/'status.json',dict(state='infrastructure_hold',error=repr(e)));raise
    else:
        s=summary(root);print(json.dumps({k:v for k,v in s.items() if k!='pairs'},indent=2))


if __name__=='__main__':main()
