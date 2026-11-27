"""Adopt the active observer without restarting it; allow a slow VM to finish.

The original policy, worlds, and 600-second ROS mission limits stay unchanged.
Only the outer host-time supervisor bound is increased to 2400 seconds.
"""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import psutil
import run as campaign

HERE=Path(__file__).resolve().parent

def finish(root,run,runs,env):
    try:
        assert (run/'metrics.json').exists(), 'Observer ended without metrics; infrastructure investigation required'
        with (run/'observer.log').open('a') as log:
            subprocess.run(['python3',str(Path(env['CAMERA_BENCH_HELPERS'])/'2026-11-19_corridor_box/finish_trial.py'),str(run),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
            subprocess.run(['python3',str(campaign.BASE.parent/'2026-11-20_v12_camera_rate/refresh_final.py'),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
    finally:campaign.cleanup(run)
    runs.append(str(run));(root/'runs.json').write_text(json.dumps(runs,indent=2)+'\n')
    report=campaign.summary.analyze(root);(root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    row=report['runs'][-1];print('DONE',row['seed'],row['strict_saved_map_success'],round(row['elapsed_sim_s'],1),flush=True)

def environment(run):
    return dict(os.environ,BENCH_ROOT=str(run),BENCH_RUN=str(run),BENCH_POLICY='mixed',
        CAMERA_VARIANT='sync',CAMERA_PROFILE='balanced',CAMERA_POLICY_WORKSPACE=str(Path.home()/'flow_v151_exposure_sync_r2_ws'),
        CAMERA_BENCH_WIDTH='480',CAMERA_BENCH_HEIGHT='480',CAMERA_BENCH_FPS='5',
        ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros_logs'))

def main(root,pid,created):
    locks=[open(Path.home()/'track-recordings'/n,'a') for n in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX)
    current=Path.home()/'corridor-board-73bbb05/recordings/v151-boxes5-track-m112601-trial1'
    observer=psutil.Process(pid)
    assert abs(observer.create_time()-created)<.01
    assert observer.environ().get('BENCH_RUN')==str(current)
    assert observer.environ().get('ROS_DOMAIN_ID')=='174'
    state=dict(pid=os.getpid(),adopted_observer_pid=pid,original_observer_continues=True,
        mission_limit_sim_s=600,outer_limit_wall_s=2400,reason='Slow VM could exceed old 1200-wall-second guard',policy_changed=False)
    (root/'supervisor_revision.json').write_text(json.dumps(state,indent=2)+'\n');print('ADOPTED',state,flush=True)
    start=time.monotonic();runs=[]
    while True:
        try:
            if not observer.is_running() or observer.status()==psutil.STATUS_ZOMBIE:break
        except psutil.NoSuchProcess:break
        if time.monotonic()-start>2400:raise RuntimeError('Adopted observer wall-time guard exceeded')
        time.sleep(1)
    finish(root,current,runs,environment(current))
    for index,seed in enumerate(range(112602,112606),2):
        fixture=root/f'map-{seed}';fixture.mkdir()
        reference=Path.home()/'track-reference-73bbb05'
        old=Path.home()/'track-resume-5e95333/2026-11-19_random_track/prepare.py'
        env=dict(os.environ,PYTHONPATH=str(reference)+':'+os.environ.get('PYTHONPATH',''))
        subprocess.run([sys.executable,str(old),str(root/'reference5'),str(fixture),'--seed',str(seed)],env=env,check=True)
        scene=json.loads((fixture/'scene.json').read_text());assert scene['room_size']==[20.15,11.35] and len(scene['obstacles'])==5
        trial=Path.home()/'corridor-board-73bbb05/recordings'/f'v151-boxes5-track-m{seed}-trial{index}';trial.mkdir()
        alias=Path.home()/'track-recordings'/trial.name;alias.symlink_to(trial,target_is_directory=True)
        for name in ('scene.json','scene.yaml','world','run'):(trial/name).symlink_to(fixture/name,target_is_directory=(fixture/name).is_dir())
        (trial/'policy_revision.txt').write_text('Mixed v1.5.1-balanced; frozen flow_v151_exposure_sync_r2_ws\n')
        env=environment(trial);print('START',index,trial,flush=True)
        try:
            with (trial/'observer.log').open('w') as log:
                subprocess.run(['bash','-lc','source /opt/ros/jazzy/setup.bash && source "$CAMERA_POLICY_WORKSPACE/install/setup.bash" && exec python3 -u "'+str(HERE/'trial.py')+'"'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=2400)
        except BaseException:
            campaign.cleanup(trial);raise
        finish(root,trial,runs,env)
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=5))+'\n')
    (root/'report.json').write_text(json.dumps(campaign.summary.analyze(root),indent=2)+'\n')
    print('COMPLETE',root,flush=True)

if __name__=='__main__':main(Path(sys.argv[1]),int(sys.argv[2]),float(sys.argv[3]))
