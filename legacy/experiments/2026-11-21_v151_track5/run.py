"""Five sequential large demo-track trials, frozen v1.5.1, independent judge."""
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'2026-11-21_camera_aware_policy'
sys.path.insert(0,str(BASE))
from campaign import cleanup
spec=importlib.util.spec_from_file_location('track_summary',BASE/'summarize.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)

def main():
    locks=[open(Path.home()/'track-recordings'/n,'a') for n in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    root=Path.home()/('v151-track-five-'+str(time.time_ns()));root.mkdir()
    reference=Path.home()/'track-reference-73bbb05'
    old=Path.home()/'track-resume-5e95333/2026-11-19_random_track/prepare.py'
    seeds=list(range(112501,112506));runs=[]
    (root/'protocol.json').write_text(json.dumps(dict(seeds=seeds,version='v1.5.1-balanced',
        room_m=[20.15,11.35],maximum_mission_s=600,camera=[480,480,5],
        wall_markers=1,box_markers=2,domain=174,physical_drive=False,
        selection='five predetermined seeds, no retry of policy failure'),indent=2)+'\n')
    print('ROOT',root,flush=True)
    for index,seed in enumerate(seeds,1):
        fixture=root/f'map-{seed}';fixture.mkdir()
        env=dict(os.environ,PYTHONPATH=str(reference)+':'+os.environ.get('PYTHONPATH',''))
        subprocess.run([sys.executable,str(old),str(reference),str(fixture),'--seed',str(seed)],env=env,check=True)
        scene=json.loads((fixture/'scene.json').read_text())
        assert scene['room_size']==[20.15,11.35]
        run=Path.home()/'corridor-board-73bbb05/recordings'/f'v151-track-m{seed}-trial{index}'
        run.mkdir()
        # The existing board resolves '-track-' media through track-recordings.
        alias=Path.home()/'track-recordings'/run.name
        if not alias.is_symlink() and not alias.exists():alias.symlink_to(run,target_is_directory=True)
        assert alias.resolve()==run.resolve()
        for name in ('scene.json','scene.yaml','world','run'):(run/name).symlink_to(fixture/name,target_is_directory=(fixture/name).is_dir())
        env=dict(os.environ,BENCH_ROOT=str(run),BENCH_RUN=str(run),BENCH_POLICY='mixed',
            CAMERA_VARIANT='sync',CAMERA_PROFILE='balanced',CAMERA_POLICY_WORKSPACE=str(Path.home()/'flow_v151_exposure_sync_r2_ws'),
            CAMERA_BENCH_WIDTH='480',CAMERA_BENCH_HEIGHT='480',CAMERA_BENCH_FPS='5',
            ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros_logs'))
        (run/'policy_revision.txt').write_text('Mixed v1.5.1-balanced; frozen flow_v151_exposure_sync_r2_ws\n')
        print('START',index,run,flush=True)
        try:
            with (run/'observer.log').open('w') as log:
                subprocess.run(['bash','-lc','source /opt/ros/jazzy/setup.bash && source "$CAMERA_POLICY_WORKSPACE/install/setup.bash" && exec python3 -u "'+str(HERE/'trial.py')+'"'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1200)
            with (run/'observer.log').open('a') as log:
                subprocess.run(['python3',str(Path(env['CAMERA_BENCH_HELPERS'])/'2026-11-19_corridor_box/finish_trial.py'),str(run),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
                subprocess.run(['python3',str(BASE.parent/'2026-11-20_v12_camera_rate/refresh_final.py'),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
        finally:cleanup(run)
        runs.append(str(run));(root/'runs.json').write_text(json.dumps(runs,indent=2)+'\n')
        report=summary.analyze(root);(root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        last=report['runs'][-1];print('DONE',index,last['strict_saved_map_success'],round(last['elapsed_sim_s'],1),flush=True)
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=5))+'\n')
    report=summary.analyze(root);(root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('COMPLETE',root,flush=True)

if __name__=='__main__':main()
