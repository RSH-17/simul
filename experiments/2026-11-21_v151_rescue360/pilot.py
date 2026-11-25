"""A real Nav2 full-spin smoke test with a 30s transition, then normal 360s trials."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-21_camera_aware_policy'))
from campaign import execute,WORKSPACES


def main():
    root=Path.home()/'rescue360-pilot1121'
    root.mkdir(exist_ok=False)
    baseline=Path.home()/'v151-track-five-boxes5-1795247134501974510'
    (root/'map-112601').symlink_to(baseline/'map-112601',target_is_directory=True)
    locks=[open(Path.home()/'track-recordings'/n,'a') for n in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX)
    WORKSPACES['rescue360']=Path.home()/'flow_v151_rescue360_r2_ws'
    os.environ['CAMERA_BENCH_TRIAL_SCRIPT']=str(HERE/'pilot_observer.py')
    run=execute(root,'rescue360','balanced',112601,'pilot1121',150)
    events=[json.loads(line) for line in (run/'native/rescue360.jsonl').read_text().splitlines()]
    success=[row for row in events if row['event']=='full_sweep_result' and row['completed']]
    report=dict(protocol='Integration smoke test only. Same first world; phase transition accelerated to 30s.',
                run=str(run),full_sweeps=success,physical_drive=False,
                confirmed=bool(success),formal_transition_s=360.)
    (root/'pilot_result.json').write_text(json.dumps(report,indent=2)+'\n')
    assert success,'No actual full sweep with exposure-time camera evidence'
    print('PILOT_VERIFIED',run.name,len(success),flush=True)
    for lock in locks:lock.close()
    # The formal run starts afresh with the unmodified 360-second parameter.
    env=dict(os.environ)
    env.pop('CAMERA_BENCH_TRIAL_SCRIPT',None)
    with (Path.home()/'rescue360-five1121.log').open('w') as stream:
        child=subprocess.Popen(['bash',str(HERE/'run.sh'),str(baseline)],env=env,stdout=stream,
                               stderr=subprocess.STDOUT,start_new_session=True,stdin=subprocess.DEVNULL)
    (root/'formal_parent_pid.txt').write_text(str(child.pid)+'\n')


if __name__=='__main__':main()
