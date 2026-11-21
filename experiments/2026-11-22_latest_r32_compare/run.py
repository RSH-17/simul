"""Opt-in development regression on unchanged worlds from the five-box baseline."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

# Reuse the original observer, launch, map fixtures and strict evaluator.

HERE = Path(__file__).resolve().parent
BASE = HERE.parent/'2026-11-21_camera_aware_policy'
sys.path.insert(0, str(BASE))
from campaign import cleanup
spec = importlib.util.spec_from_file_location('wall_transit_summary', BASE/'summarize.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)
analyze = summary.analyze


def main(baseline, seeds):
    assert os.environ['ROS_DOMAIN_ID'] == '174'
    assert (baseline/'complete.json').exists()
    assert len(seeds) == len(set(seeds))
    locks = [open(Path.home()/'track-recordings'/n, 'a') for n in ('.domain.lock', '.second-domain.lock')]
    for lock in locks:
        fcntl.flock(lock, fcntl.LOCK_EX)
    workspace = Path.home()/'r32_upstream_d8704f6_ws'
    assert (workspace/'install/setup.bash').exists()
    root = Path.home()/('r32-latest-d8704f6-'+str(time.time_ns()))
    root.mkdir()
    protocol = dict(version='upstream-r32-d8704f6', baseline=str(baseline), seeds=seeds,
        split='development_regression', maximum_mission_sim_s=600, outer_wall_limit_s=2400,
        frozen_policy=True, source_manifest=json.loads((workspace/'source_manifest.json').read_text()),
        camera=[480, 480, 5], physical_drive=False, domain=174,
        upstream_commit='d8704f64d7b9098c79a9c42dfec81e29d6d789a8',
        common_judge='registered then independent 1.5m, fixed-frame map 0.25m, zero bumps; no Mixed-only witness required',
        original_navigation_profile=True,
        policy_inputs='live sensors and SLAM only; no evaluator truth', retry_failed_policy=False)
    (root/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    print('ROOT', root, flush=True)
    runs = []
    for seed in seeds:
        fixture = baseline/f'map-{seed}'
        assert len(json.loads((fixture/'scene.json').read_text())['obstacles']) == 5
        (root/f'map-{seed}').symlink_to(fixture, target_is_directory=True)
        run = Path.home()/'corridor-board-73bbb05/recordings'/f'r32-latest-d8704f6-track-m{seed}-{root.name.split("-")[-1]}'
        run.mkdir()
        (Path.home()/'track-recordings'/run.name).symlink_to(run, target_is_directory=True)
        for name in ('scene.json', 'scene.yaml', 'world', 'run'):
            (run/name).symlink_to(fixture/name, target_is_directory=(fixture/name).is_dir())
        (run/'policy_revision.txt').write_text('Original R32 latest d8704f64 · common 1.5m judge\n')
        import yaml
        original=workspace/'src/osp_project/resources/mission_r32_hardware.yaml'
        settings=yaml.safe_load(original.read_text())
        settings['mission']['ros__parameters'].update(mission_time_source='ros',duration=600.)
        settings['recorder']['ros__parameters']['record_drive_evidence']=False
        (run/'r32-sim.yaml').write_text(yaml.safe_dump(settings))
        env = dict(os.environ, BENCH_ROOT=str(run), BENCH_RUN=str(run), BENCH_POLICY='r32',
            CAMERA_VARIANT='region_wall', CAMERA_PROFILE='balanced',
            CAMERA_POLICY_WORKSPACE=str(Path.home()/'flow_v151_exposure_sync_r2_ws'), CAMERA_BENCH_WIDTH='480',
            CAMERA_BENCH_HEIGHT='480', CAMERA_BENCH_FPS='5',
            ROS_HOME=str(run/'ros_home'), ROS_LOG_DIR=str(run/'ros_logs'))
        before = hashlib.sha256((fixture/'world/osp_studio.sdf').read_bytes()).hexdigest()
        print('START', seed, run, flush=True)
        try:
            with (run/'observer.log').open('w') as log:
                subprocess.run(['bash', '-lc', 'source /opt/ros/jazzy/setup.bash && source "'+
                    str(workspace/'install/local_setup.bash')+'" && exec python3 -u "'+str(HERE/'trial.py')+'"'],
                    env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=2400)
            with (run/'observer.log').open('a') as log:
                subprocess.run(['python3', str(Path(env['CAMERA_BENCH_HELPERS'])/
                    '2026-11-19_corridor_box/finish_trial.py'), str(run), str(run)],
                    env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
        except BaseException as exc:
            (run/'trial_error.json').write_text(json.dumps(dict(error=repr(exc),
                completed_comparison_sample=False, classification='needs_investigation'), indent=2)+'\n')
            raise
        finally:
            cleanup(run)
        assert before == hashlib.sha256((fixture/'world/osp_studio.sdf').read_bytes()).hexdigest()
        runs.append(str(run))
        (root/'runs.json').write_text(json.dumps(runs, indent=2)+'\n')
        metrics=json.loads((run/'metrics.json').read_text())
        print('DONE',seed,metrics['final_ids'],round(metrics['elapsed_sim_s'],1),flush=True)
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=len(runs)))+'\n')
    print('COMPLETE', root, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('baseline', type=Path)
    p.add_argument('--seeds', type=int, nargs='+', default=list(range(112601,112606)), choices=range(112601, 112606))
    a = p.parse_args()
    main(a.baseline.resolve(), a.seeds)
