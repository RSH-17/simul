"""Single-native, fixed-plan comparison on archived tracks and unseen geometries."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import psutil

from fixtures import make

HERE = Path(__file__).resolve().parent
HOME = Path.home()
EXPERIMENTS = Path(os.environ['SURVEY_EXPERIMENTS'])
sys.path.insert(0, str(EXPERIMENTS/'2026-11-21_camera_aware_policy'))
from summarize import analyze
from campaign import cleanup
ARCHIVE = HOME / 'v151-track-five-boxes5-1795247134501974510'
BASELINE = HOME / 'flow_v151_exposure_sync_r2_ws'
REGRESSION = (112601, 112605, 112602, 112603, 112604)
HELDOUT = (112801, 112802)


def write(path, data):
    tmp = path.with_suffix('.partial')
    tmp.write_text(json.dumps(data, indent=2)+'\n')
    tmp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def guard():
    if os.environ['ROS_DOMAIN_ID'] != '174' or os.environ['ROS_LOCALHOST_ONLY'] != '1':
        raise RuntimeError('Requires the isolated local simulation domain.')
    for proc in psutil.process_iter():
        try:
            env = proc.environ()
            if env.get('BENCH_RUN') and env.get('ROS_DOMAIN_ID') in ('173','174','175'):
                raise RuntimeError('Another native benchmark is active: '+env['BENCH_RUN'])
        except psutil.Error:
            pass


def setup(root):
    root.mkdir()
    fixtures = root/'fixtures'
    fixtures.mkdir()
    worlds = {seed: make(fixtures,seed) for seed in HELDOUT}
    baseline_runs = [Path(p) for p in json.loads((ARCHIVE/'runs.json').read_text())]
    for seed in REGRESSION:
        fixture = ARCHIVE/f'map-{seed}'
        (fixtures/f'map-{seed}').symlink_to(fixture, target_is_directory=True)
        worlds[seed] = digest(fixture/'world/osp_studio.sdf')
    write(root/'protocol.json', dict(baseline_archive=str(ARCHIVE), archived_runs=list(map(str,baseline_runs)),
         regression=REGRESSION, heldout=HELDOUT, worlds=worlds, budget_sim_s=600,
         camera=[480,480,5], physical_drive=False, collision_profile='unchanged balanced',
         heldout_selection='two new fixed topologies, hidden from policy developers until candidate freeze',
         definition='strict original judge, saved map <=0.25m, physical approach <=1.5m, required ID witness, zero bumps',
         policy_failure_retried=False))


def trial(root, role, seed, workspace):
    test = 'heldout' if seed in HELDOUT else 'track'
    name = f'v151-survey-{role}-{test}-m{seed}-{root.name.split("-")[-1]}'
    run = HOME/'corridor-board-73bbb05/recordings'/name
    run.mkdir()
    (HOME/'track-recordings'/name).symlink_to(run, target_is_directory=True)
    fixture = root/'fixtures'/f'map-{seed}'
    for key in ('scene.json','scene.yaml','world','run'):
        if (fixture/key).exists():
            (run/key).symlink_to(fixture/key,target_is_directory=(fixture/key).is_dir())
    expected_hash = json.loads((root/'protocol.json').read_text())['worlds'][str(seed)]
    if digest(run/'world/osp_studio.sdf') != expected_hash:
        raise RuntimeError('Frozen world hash changed')
    source = workspace/'src/osp_mission'
    if not source.is_dir() or not (workspace/'install/setup.bash').exists():
        raise FileNotFoundError(workspace)
    write(run/'policy_manifest.json', {str(p.relative_to(source)):digest(p) for p in sorted(source.rglob('*'))
          if p.is_file() and p.suffix in ('.py','.yaml','.json') and '__pycache__' not in p.parts})
    (run/'policy_revision.txt').write_text(f'Mixed v1.5.1 {role} · {workspace.name}\n')
    env = dict(os.environ, BENCH_ROOT=str(run), BENCH_RUN=str(run), BENCH_POLICY='mixed',
               CAMERA_VARIANT=role, CAMERA_PROFILE='balanced', CAMERA_POLICY_WORKSPACE=str(workspace),
               CAMERA_BENCH_WIDTH='480', CAMERA_BENCH_HEIGHT='480', CAMERA_BENCH_FPS='5',
               ROS_HOME=str(run/'ros_home'), ROS_LOG_DIR=str(run/'ros_logs'))
    print('START',role,seed,run,flush=True)
    try:
        with (run/'observer.log').open('w') as log:
            subprocess.run(['bash','-c','source /opt/ros/jazzy/setup.bash && source "$1/install/setup.bash" && exec python3 -u "$2"',
                            'trial',str(workspace),str(HERE/'trial.py')], env=env, stdout=log, stderr=subprocess.STDOUT,
                           check=True, timeout=2400)
            subprocess.run([sys.executable,str(Path(env['CAMERA_BENCH_HELPERS'])/'2026-11-19_corridor_box/finish_trial.py'),
                            str(run),str(run)], env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
            subprocess.run([sys.executable,str(EXPERIMENTS/'2026-11-20_v12_camera_rate/refresh_final.py'),str(run)],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
    except BaseException as exc:
        write(run/'interruption.json',dict(reason=str(exc), complete=False))
        raise
    finally:
        cleanup(run)
    if digest(run/'world/osp_studio.sdf') != expected_hash:
        raise RuntimeError('Fixture changed while running')
    return run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('role',choices=('baseline','candidate'))
    parser.add_argument('--workspace',type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    locks = [open(HOME/'track-recordings'/n,'a') for n in ('.domain.lock','.second-domain.lock')]
    for lock in locks:
        fcntl.flock(lock,fcntl.LOCK_EX)
    guard()
    if args.role == 'baseline' and not root.exists():
        setup(root)
    if not (root/'protocol.json').is_file():
        parser.error('Run baseline setup first')
    if args.role == 'candidate' and args.workspace is None:
        parser.error('Candidate workspace required')
    workspace = args.workspace.resolve() if args.workspace else BASELINE
    phase = root/args.role
    phase.mkdir(exist_ok=True)
    saved = phase/'runs.json'
    rows = json.loads(saved.read_text()) if saved.exists() else []
    seeds = HELDOUT if args.role == 'baseline' else (*REGRESSION[:2],*HELDOUT,*REGRESSION[2:])
    for index, seed in enumerate(seeds):
        if index < len(rows):
            if f'-m{seed}-' not in Path(rows[index]).name:
                raise RuntimeError('Saved trials differ from registered order')
            continue
        guard()
        completed = trial(root,args.role,seed,workspace)
        rows.append(str(completed))
        write(saved,rows)
        report = analyze(phase)
        write(phase/'report.json',report)
        row = report['runs'][-1]
        print('DONE',args.role,seed,row['registered_ids'],row['strict_saved_map_success'],round(row['elapsed_sim_s'],1),flush=True)
    write(phase/'complete.json',dict(complete=True,count=len(seeds)))
    write(phase/'report.json',analyze(phase))
    print('COMPLETE',phase,flush=True)


if __name__ == '__main__':
    main()
