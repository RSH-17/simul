"""Launch one independently scored random exam from the shared Jazzy simulation board."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import secrets
import shutil
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
HOME=Path.home()
TRACK=HOME/'track-recordings'
MODES={
    'exam-official-balanced':('baseline','Balanced · 실제 시험 조건',3,'fixed','flow_v151_exposure_sync_r2_ws'),
    'exam-official-print':('candidate','Balanced Print · 실제 시험 조건',3,'fixed','v151_balanced_print_r1_1122_ws'),
    'exam-official-survey':('visualsurvey','Mixed v1.5.2 Visual Survey · 벽·군집 탐색',3,'fixed','v152_visual_survey_r3_1122_ws'),
    'exam-balanced':('baseline','Balanced · 책상 재배치',3,'shuffle','flow_v151_exposure_sync_r2_ws'),
    'exam-print':('candidate','Balanced Print · 책상 재배치',3,'shuffle','v151_balanced_print_r1_1122_ws'),
    'exam-print-unknown':('candidate','Balanced Print · 책상 재배치·개수 미공개',0,'shuffle','v151_balanced_print_r1_1122_ws'),
}


def reserve(slots, track):
    for filename in ('.domain.lock','.second-domain.lock'):
        handle=slots.enter_context((track/filename).open('a'))
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)


def prepare(stage, track, seed, count, reference, furniture='fixed'):
    import numpy as np
    from fixture import FILES, FIXED_TABLE_ROOM, architecture, build, digest, read
    import xml.etree.ElementTree as ET
    fixtures=stage/'fixtures'
    fixtures.mkdir()
    if reference:
        ref=Path(reference)
        if ref.parent!=track or ref.name in ('.','..') or '-track-' not in ref.name:
            raise ValueError('Choose a recording on this simulation board.')
        scene=read(ref/'scene.json')
        if scene.get('randomization',{}).get('generator')!='exam-random-layout':
            raise ValueError('Choose a randomized exam recording for exact replay.')
        seed=scene['seed']
        targets=scene['targets']
        if count and len(targets)!=count:
            raise ValueError('Choose the undisclosed-count policy to replay this recording.')
        if scene['randomization'].get('furniture')!=furniture:
            raise ValueError('Choose a recording with the same desk layout mode.')
        if furniture=='fixed':
            anchor=track/'.layouts/uFjtqfyJ'
            if (digest(ref/FILES[3])!=digest(anchor/FILES[3]) or
                architecture(ET.parse(ref/FILES[0]).getroot(),scene,True)!=FIXED_TABLE_ROOM):
                raise ValueError('The recorded furniture differs from the original exam room.')
        (fixtures/f'map-{seed}').symlink_to(ref.resolve(),target_is_directory=True)
        return dict(seed=seed,tables=furniture,marker_count=len(targets),start=[scene['robot_spawn'][k] for k in ('x','y','yaw')],
                    files={name:digest(ref/name) for name in FILES},replayed_recording=ref.name)
    seed=seed if seed is not None else secrets.randbits(32)
    if not 0<=seed<2**32:
        raise ValueError('Expected an unsigned 32-bit seed.')
    total=count or int(np.random.default_rng([seed,0x544234]).integers(1,6))
    return build(track/'.layouts/uFjtqfyJ',fixtures/f'map-{seed}',seed,total,furniture)


def finish_card(run, cards, url, score, title, timeout=55):
    from board_audit import reconcile
    from board_results import apply
    from urllib.error import HTTPError, URLError
    deadline=time.monotonic()+timeout
    while True:
        try:
            state=reconcile(run,cards,url)
        except HTTPError as exc:
            if exc.code not in (404,503):
                raise
            state=f'HTTP {exc.code}'
        except (URLError, TimeoutError) as exc:
            state=str(exc)
        if state in ('current','cache refreshed') and apply(cards/run.name,{'score':score},title):
            return
        if time.monotonic()>=deadline:
            raise RuntimeError(f'The scored recording is saved at {run}. The board is still waiting for {state}.')
        time.sleep(1)


def execute(stage, args):
    from fixture import read, save
    from scoring import score
    old=HERE.parent/'2026-11-22_exam_track_validation'
    engine=HERE.parent/'2026-11-22_v151_adaptive_survey'
    sys.path[:0]=[str(old),str(engine/'validation'),str(engine)]
    import paired
    from validate import identity, measure
    import run as runner
    role,title,count,furniture,workspace=MODES[args.mode]
    ws=HOME/workspace
    profile=identity(ws)
    if role=='visualsurvey':
        survey=read(HERE/'visual_survey/enrollment.json')
        approved=survey['source_manifest_sha256']
        os.environ['OSP_SURVEY_DISTANT_HINTS']='true' if survey['board_distant_hints'] else 'false'
    else:
        approved=read(HERE/'balanced_print/enrollment.json')['policies'][role]
    if profile['source_manifest_sha256']!=approved:
        raise ValueError('The installed simulation policy differs from the tested version.')
    if not args.check:runner.guard()
    reference=os.environ.get('TRACK_REPLAY_DIR','')
    world=prepare(stage,TRACK,args.seed,count,reference,furniture)
    fixture=stage/'fixtures'/f"map-{world['seed']}"
    paired.checked_files(fixture,world)
    print(json.dumps(dict(policy=title,seed=world['seed'],public_marker_count=count,furniture=furniture,
                          check_only=args.check,physical_drive=False,
                          source_sha256=approved,world_sha256=world['files']['world/osp_studio.sdf'])),flush=True)
    if args.check:return
    save(stage/'exam.json',dict(mode=args.mode,policy=profile,world=world,public_marker_count=count))
    save(stage/'protocol.json',dict(worlds={str(world['seed']):world['files']['world/osp_studio.sdf']}))
    runner.HERE=HERE
    runner.HELDOUT=()
    os.environ['OSP_PUBLIC_MARKER_COUNT']=str(count)
    run=runner.trial(stage,args.mode,world['seed'],ws)
    result=score(paired.adjudicate(run,world,profile,measure),read(run/'metrics.json'),read(run/'scene.json'))
    save(stage/'score.json',result)
    save(run/'exam-score.json',result)
    cards=Path(os.environ['CAMERA_BENCH_CARDS'])
    finish_card(run,cards,paired.BOARD,result,title)
    print(json.dumps(dict(run=run.name,strict=result['strict'],registered=result['registered'],
                          verified=result['verified'],elapsed_s=result['elapsed_s'])),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=MODES)
    parser.add_argument('--seed',type=int)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    with ExitStack() as slots:
        if args.check:
            stage=Path(slots.enter_context(tempfile.TemporaryDirectory()))
        else:
            try:reserve(slots,TRACK)
            except BlockingIOError:
                print('A native simulator is already reserved.',flush=True)
                return 75
            if shutil.disk_usage(HOME).free<3*1024**3:
                raise RuntimeError('Insufficient space for a full native camera recording.')
            stage=HOME/'random-exam-board/runs'/f'run-{time.time_ns()}'
            stage.mkdir(parents=True,exist_ok=False)
        execute(stage,args)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
