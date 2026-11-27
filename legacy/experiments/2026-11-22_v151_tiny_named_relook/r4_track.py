"""Finite development comparison, limited to the five original large demo tracks."""
import argparse
from contextlib import ExitStack, contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

from track_screen import ORDER, accepted_glances, common_maps, judge, predecessor

HERE=Path(__file__).resolve().parent
ADAPTIVE=HERE.parent/'2026-11-22_v151_adaptive_survey'
sys.path[:0]=[str(ADAPTIVE/'validation'),str(ADAPTIVE)]
HOME=Path.home()
ARCHIVE=HOME/'v151-track-five-boxes5-1795247134501974510'
R2=HOME/'v151-adaptive-survey-paired-1122'
R3=HOME/'v151-object-followthrough-screen-112203'
ROOT=HOME/'v151-tiny-named-relook-track-112204'
RECORDINGS=HOME/'corridor-board-73bbb05/recordings'
BASE=HOME/'flow_v151_exposure_sync_r2_ws'
BASE_ID='82410a7721c0b6dca32e95e5e9d43c30537360dd29a8f579ecbc0d53bd31317a'
R3_SOURCE='2a1be84b626436e3db3060bec0fd3304d268d7885ac3af7066e3ad7e2edfc243'
R3_MANIFEST='9071751d693de94db49017ae843317b6992903d1286e6245ff86b16c4b09c75e'


def read(file):
    return json.loads(file.read_text())


def write(file,data):
    temp=file.with_name(file.name+'.new')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temp.replace(file)


def digest(file):
    with file.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def stable(data):
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def predecessor_state():
    queue=HOME/'v151-object-followthrough-queue1122.json'
    def complete_seven(root):
        f=root/'candidate/complete.json'
        if not f.is_file():
            return False
        status=read(f)
        return status.get('complete') is True and status.get('count')==7
    futility=R3/'futility_stop.json'
    return predecessor(read(queue)['state'] if queue.is_file() else None,complete_seven(R2),
                       futility.is_file() and read(futility).get('finished_cases')==2,
                       (R3/'seven_case_gate.json').is_file() and complete_seven(R3))


@contextmanager
def native_locks(directory):
    with ExitStack() as stack:
        for name in ('.domain.lock','.second-domain.lock'):
            fd=stack.enter_context((directory/name).open('a'))
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield


def receipt(ws, replay, jazzy):
    from validate import identity
    ws=ws.resolve()
    child=read(ws/'tiny_named_manifest.json')
    if (child['version']!='1.5.1-tiny-named-relook' or child['parent_manifest_sha256']!=R3_MANIFEST
            or digest(ws/'followthrough_manifest.json')!=R3_MANIFEST or not child['child_modules']):
        raise ValueError('Unexpected staged R4 or R3 parent')
    for name,sha in child['child_modules'].items():
        if Path(name).name!=name or digest(ws/'src/osp_mission/osp_mission'/name)!=sha:
            raise ValueError('Changed tiny_named_manifest module')
    identity_now=identity(ws)
    if stable(identity(BASE))!=BASE_ID or any(identity_now['installed_python'].get(k)!=v for k,v in child['child_modules'].items()):
        raise ValueError('Baseline or installed R4 differs from frozen source')
    return dict(version=child['version'],workspace=str(ws),policy_manifest_sha256=digest(ws/'tiny_named_manifest.json'),
                workspace_identity_sha256=stable(identity_now),source_manifest_sha256=identity_now['source_manifest_sha256'],
                baseline_identity_sha256=BASE_ID,
                replay={'path':str(replay.resolve()),'sha256':digest(replay)},
                jazzy={'path':str(jazzy.resolve()),'sha256':digest(jazzy)},
                replay_passed=False,jazzy_passed=False)


def check_receipt(file,pinned):
    if digest(file)!=pinned:
        raise ValueError('Approval receipt SHA differs from the explicitly supplied SHA')
    supplied=read(file)
    current=receipt(Path(supplied['workspace']),Path(supplied['replay']['path']),Path(supplied['jazzy']['path']))
    for key in ('replay_passed','jazzy_passed'):
        if supplied.get(key) is not True:
            raise ValueError('Missing reviewed result for '+key)
        current[key]=True
    if current!=supplied:
        raise ValueError('Reviewed source, stage, or evidence changed')
    return current


def archived_paths(folder, allowed):
    listed={Path(p).name:Path(p) for p in read(folder/'runs.json')}
    result={}
    for row in read(folder/'report.json')['runs']:
        s=row['seed']
        if s not in allowed:
            continue
        run=listed[row['run']]
        if s in result or run.resolve()!=run or run.parent!=RECORDINGS:
            raise ValueError('Incorrect or duplicate demo track recording')
        result[s]=(run,row)
    return result


def fixture_hashes():
    from rescreen import frozen_world_and_pose
    expected=read(R2/'protocol.json')['worlds']
    files=frozen_world_and_pose(R2,{'worlds':{str(s):expected[str(s)] for s in ORDER}})
    for s in ORDER:
        fixture=R2/'fixtures'/f'map-{s}'
        if fixture.resolve()!=(ARCHIVE/f'map-{s}').resolve():
            raise ValueError('Development fixture is not the original demo track')
        demo='run/simulation/demo_layout.json'
        resources=[p for p in (fixture/'world').rglob('*') if p.is_file() and p.suffix.lower() in
                   {'.dae','.obj','.stl','.mtl','.png','.jpg','.mesh','.material','.sdf'}]
        files[str(s)].update({str(p.relative_to(fixture)):digest(p) for p in resources})
        files[str(s)][demo]=digest(fixture/demo)
    return files


def measurements(run,row,seed,sha,source=None):
    from compare import measurements as original
    from validate import measure
    if row['seed']!=seed or row['run']!=run.name:
        raise ValueError('Run and report disagree')
    old=original(run,row)
    if sha!=old['sha256']:
        raise ValueError('Recorded world does not match original demo track')
    if source and stable(read(run/'policy_manifest.json'))!=source:
        raise ValueError('Recorded source does not match frozen policy')
    raw=measure(run,{'files':{'world/osp_studio.sdf':sha}})
    ids=set(raw['registered_ids']) & set(old['final_ids'])
    near=set(raw['after_registration_near_s']) & set(old['verified_near_ids'])
    with (run/'launch.log').open(errors='replace') as f:
        cue=accepted_glances(f)
    ret=dict(run=run.name,valid=raw['valid'],elapsed_s=raw['elapsed_s'],world_sha256=sha,
             configured_start=old['configured_start'],settled_xy=raw['settled_xy'],wall_ids=old['wall_ids'],box_ids=old['box_ids'],
             strict=raw['strict'] and old['strict'],original_strict=old['strict'],raw_strict=raw['strict'],
             registered_ids=sorted(ids),near_ids=sorted(near),raw_near_time_s=raw['after_registration_near_s'],
             unexpected_ids=raw['unexpected_ids'],bumps=raw['bumps'],distance_m=raw['distance_m'],
             elapsed_registration_s=read(run/'metrics.json')['registered_s'],map_errors_m=raw['map_errors_m'],
             run_source_sha256=source,**cue)
    for tag in ('wall','box'):
        hosts=set(old[tag+'_ids'])
        ret[tag+'_reg']=len(ids & hosts)
        ret[tag+'_near']=len(near & hosts)
    return ret


def claim(root,seed):
    path=root/f'attempt-{seed}.json'
    with path.open('x') as stream:
        json.dump({'seed':seed,'state':'claimed_once'},stream)


def report(root, admission, hashes):
    refs=archived_paths(ARCHIVE,ORDER)
    observed=archived_paths(root/'candidate',ORDER)
    pairs=[]
    for seed in ORDER:
        if seed not in observed:
            continue
        sha=hashes[str(seed)]['world/osp_studio.sdf']
        a=measurements(*refs[seed],seed,sha)
        b=measurements(*observed[seed],seed,sha,admission['source_manifest_sha256'])
        pairs.append(dict(seed=seed,baseline=a,candidate=b))
    r4={p['seed']:p['candidate'] for p in pairs}
    r3={}
    if (R3/'candidate/report.json').exists():
        for seed,(run,row) in archived_paths(R3/'candidate',set(r4)).items():
            r3[seed]=measurements(run,row,seed,hashes[str(seed)]['world/osp_studio.sdf'],R3_SOURCE)
    early=judge(pairs[:2],False) if len(pairs)>=2 else None
    final=judge(pairs,True) if len(pairs)==len(ORDER) else None
    result=dict(scope='five original 8780 demo-track development seeds only',pairs=pairs,first_two=early,
                five_track=final,r3_vs_r4_common=common_maps(r3,r4),automatic_promotion=False,
                limits=['archived balanced has release evidence but no per-run source hash',
                       'accepted glance is a native action request, not proof of completed rotation or causal registration'])
    write(root/'track_comparison.json',result)
    return result


def launch(admission,pin):
    import run as runner
    from supervise import disk_budget
    if predecessor_state()!='ready':
        return {'state':predecessor_state()}
    with native_locks(HOME/'track-recordings'):
        runner.guard()
        if (ROOT/'terminal.json').exists():
            if read(ROOT/'registration.json')['approval']!=pin:
                raise ValueError('Completed campaign belongs to another approval receipt')
            return read(ROOT/'terminal.json')
        hashes=fixture_hashes()
        refs=archived_paths(ARCHIVE,ORDER)
        if set(refs)!=set(ORDER):
            raise ValueError('Missing historical demo-track comparator')
        reference={str(s):{p:digest(refs[s][0]/p) for p in ('metrics.json','audit.json','events.jsonl','trajectory.csv')}
                   for s in ORDER}
        registration=dict(approval=pin,source=admission,order=list(ORDER),fixtures=hashes,
                          baseline_evidence=reference,maximum_sim_seconds=600,physical_drive=False)
        if not ROOT.exists():
            ROOT.mkdir()
            (ROOT/'fixtures').symlink_to(R2/'fixtures',target_is_directory=True)
            (ROOT/'candidate').mkdir()
            write(ROOT/'protocol.json',{'worlds':{k:v['world/osp_studio.sdf'] for k,v in hashes.items()}})
            write(ROOT/'registration.json',registration)
        if read(ROOT/'registration.json')!=registration:
            raise ValueError('Frozen enrollment, world or archived evidence changed')
        phase=ROOT/'candidate'
        paths=read(phase/'runs.json') if (phase/'runs.json').exists() else []
        expected_names=[RECORDINGS/f'v151-survey-candidate-track-m{s}-112204' for s in ORDER]
        if paths!=[str(p) for p in expected_names[:len(paths)]]:
            raise ValueError('Saved progress differs from prespecified order')
        if any((ROOT/f'attempt-{s}.json').exists() or expected_names[i].exists()
               or (HOME/'track-recordings'/expected_names[i].name).is_symlink()
               for i,s in enumerate(ORDER) if i>=len(paths)):
            state=dict(state='infrastructure_hold',reason='Prior unadjudicated attempt or conflicting card; no rerun',
                       count=len(paths))
            write(ROOT/'terminal.json',state)
            return state
        def end(state,decision):
            result=dict(state=state,decision=decision,count=len(paths))
            if len(paths)==len(ORDER) and state in ('screen_declined','five_track_screen_passed'):
                write(phase/'complete.json',dict(complete=True,count=len(paths)))
                measured=read(phase/'report.json')
                measured['complete']=True
                write(phase/'report.json',measured)
            write(ROOT/'terminal.json',result)
            return result
        def assess():
            write(phase/'report.json',runner.analyze(phase))
            result=report(ROOT,admission,hashes)
            if any(not p[role]['valid'] for p in result['pairs'] for role in ('baseline','candidate')):
                return end('infrastructure_hold','missing independent evidence on a completed trial')
            for name in ('first_two','five_track'):
                d=result[name]
                if d and d['state']!='passed':
                    return end('screen_declined' if d['state']=='declined' else 'infrastructure_hold',d)
            if result['five_track']:
                return end('five_track_screen_passed',result['five_track'])
        if paths:
            done=assess()
            if done:
                return done
        for i in range(len(paths),len(ORDER)):
            seed=ORDER[i]
            required=disk_budget(len(ORDER)-i)
            if shutil.disk_usage(ROOT).free<required:
                return {'state':'storage_hold','required_bytes':required,'remaining':len(ORDER)-i}
            runner.guard()
            if not same_identity(admission) or fixture_hashes()!=hashes:
                raise RuntimeError('Changed installed policy or track')
            claim(ROOT,seed)
            try:
                completed=runner.trial(ROOT,'candidate',seed,Path(admission['workspace']))
                if completed!=expected_names[i] or not same_identity(admission):
                    raise RuntimeError('Unexpected native result or changed running policy')
                paths.append(str(completed))
                write(phase/'runs.json',paths)
                from board_audit import reconcile
                try:
                    board=reconcile(completed,Path(os.environ['CAMERA_BENCH_CARDS']),'http://192.168.252.2:8780')
                except (OSError,RuntimeError,ValueError) as exc:
                    board='dashboard refresh unavailable: '+str(exc)
                write(ROOT/f'board-{seed}.json',{'run':completed.name,'status':board})
                done=assess()
            except (Exception,KeyboardInterrupt) as exc:
                end('infrastructure_hold',repr(exc))
                raise
            if done:
                return done
        raise RuntimeError('No final five-track decision')


def same_identity(admission):
    from validate import identity
    ws=Path(admission['workspace'])
    return (stable(identity(ws))==admission['workspace_identity_sha256'] and
            digest(ws/'tiny_named_manifest.json')==admission['policy_manifest_sha256'] and stable(identity(BASE))==BASE_ID)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    a=sub.add_parser('inspect')
    a.add_argument('workspace',type=Path);a.add_argument('replay',type=Path);a.add_argument('jazzy',type=Path)
    a=sub.add_parser('run');a.add_argument('approved_receipt',type=Path);a.add_argument('approved_sha256')
    sub.add_parser('status')
    args=p.parse_args()
    if args.action=='inspect':
        result=receipt(args.workspace,args.replay,args.jazzy)
    elif args.action=='status':
        result=read(ROOT/'terminal.json') if (ROOT/'terminal.json').exists() else {'predecessor':predecessor_state(),'completed':len(read(ROOT/'candidate/runs.json')) if (ROOT/'candidate/runs.json').exists() else 0}
    else:
        data=check_receipt(args.approved_receipt,args.approved_sha256)
        try:
            result=launch(data,args.approved_sha256)
        except BlockingIOError:
            result={'state':'shared_engine_busy'}
    print(json.dumps(result,indent=2,allow_nan=False),flush=True)
    if result.get('state') in ('infrastructure_hold','waiting_for_preceding_campaign','storage_hold','shared_engine_busy'):
        return 75
    return 0


if __name__=='__main__':
    sys.exit(main())
