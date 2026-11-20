"""Matched, sequential development comparison followed by held-out evaluation."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-20_v12_camera_rate'))
from suite import cleanup

PROFILES=dict(balanced=(480,5),glance=(480,10),detail=(640,5))
WORKSPACES=dict(baseline=Path.home()/'flow_v12_camera_bench_ws',candidate=Path.home()/'flow_v15_camera_aware_r2_ws')


def execute(root,variant,profile,seed,phase,duration):
    fixture=root/f'map-{seed}'
    run=Path.home()/'corridor-board-73bbb05/recordings'/f'v15-{variant}-{profile}-m{seed}-{phase}'
    if run.exists():raise FileExistsError(run)
    run.mkdir();(run/'world').symlink_to(fixture/'world',target_is_directory=True)
    scene=yaml.safe_load((fixture/'scene.yaml').read_text());scene['duration']=duration
    (run/'scene.yaml').write_text(yaml.safe_dump(scene));(run/'scene.json').write_bytes((fixture/'scene.json').read_bytes())
    size,fps=PROFILES[profile]
    env=dict(os.environ,BENCH_ROOT=str(run),BENCH_RUN=str(run),BENCH_POLICY='mixed',
        CAMERA_VARIANT=variant,CAMERA_PROFILE=profile,CAMERA_POLICY_WORKSPACE=str(WORKSPACES[variant]),
        CAMERA_BENCH_WIDTH=str(size),CAMERA_BENCH_HEIGHT=str(size),CAMERA_BENCH_FPS=str(fps),
        ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros_logs'))
    print('START',run.name,flush=True)
    try:
        with (run/'observer.log').open('w') as log:
            trial_script=Path(env.get('CAMERA_BENCH_TRIAL_SCRIPT',str(HERE/'strict_trial.py')))
            subprocess.run(['bash','-lc','source /opt/ros/jazzy/setup.bash && source "$CAMERA_POLICY_WORKSPACE/install/setup.bash" && exec python3 -u "'+str(trial_script)+'"'],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
        helper=Path(env['CAMERA_BENCH_HELPERS'])/'2026-11-19_corridor_box/finish_trial.py'
        with (run/'observer.log').open('a') as log:
            subprocess.run(['/usr/bin/python3',str(helper),str(run),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
            subprocess.run(['/usr/bin/python3',str(HERE.parent/'2026-11-20_v12_camera_rate/refresh_final.py'),str(run)],
                           env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
    except Exception as exc:
        (run/'trial_error.json').write_text(json.dumps(dict(error=repr(exc),excluded_from_ranking=True))+'\n')
        raise
    finally:cleanup(run)
    metrics=json.loads((run/'metrics.json').read_text())
    assert metrics['world_from_map_initial_alignment'] is not None,'Missing independent map alignment'
    print('DONE',run.name,'strict=',metrics['strict_success'],'time=',round(metrics['elapsed_sim_s'],1),flush=True)
    return run


def selection(runs):
    rows=[json.loads((Path(run)/'metrics.json').read_text()) for run in runs]
    chosen={}
    for variant in WORKSPACES:
        options=[]
        for profile in PROFILES:
            matches=[row for row in rows if row['variant']==variant and row['profile']==profile]
            size,fps=PROFILES[profile]
            score=(-sum(row['strict_success'] for row in matches),
                   sum(row['bumper_events'] for row in matches),
                   sum(row['elapsed_sim_s'] for row in matches)/len(matches),size*size*fps)
            options.append((score,profile))
        chosen[variant]=min(options)[1]
    return chosen


def main(root):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rows=[]
    for index,seed in enumerate((112021,112022)):
        for profile in PROFILES:
            for variant in (('baseline','candidate') if index%2==0 else ('candidate','baseline')):
                run=execute(root,variant,profile,seed,'dev',180)
                rows.append(str(run));(root/'runs.json').write_text(json.dumps(rows,indent=2)+'\n')
    chosen=selection(rows)
    (root/'selection.json').write_text(json.dumps(dict(profiles=chosen,basis='development maps only; no held-out measurements used'),indent=2)+'\n')
    for index,seed in enumerate((112101,112102,112103)):
        for variant in (('baseline','candidate') if index%2==0 else ('candidate','baseline')):
            run=execute(root,variant,chosen[variant],seed,'heldout',300)
            rows.append(str(run));(root/'runs.json').write_text(json.dumps(rows,indent=2)+'\n')
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=len(rows),development_runs=12,held_out_runs=6,
        profiles=chosen,physical_drive=False),indent=2)+'\n')


if __name__=='__main__':main(Path(sys.argv[1]).resolve())
