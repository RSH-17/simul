"""Record local simulator sensor timing for post-run range and navigation diagnosis."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

TOPICS=('/clock','/scan','/scan_self_filtered','/tf','/tf_static','/mission/edge_print_bearing',
        '/mission/processed_camera','/mission/marker_bearing','/mission/marker_candidates',
        '/mission/marker_observations','/mission/detector_status',
        '/navigate_to_pose/_action/status','/spin/_action/status')
TERMINAL={'complete','invalid_pair','infrastructure_hold'}


def state(root):
    path=root/'status.json'
    return json.loads(path.read_text()) if path.is_file() else {}


def monitor(root,recordings):
    if os.environ.get('ROS_DOMAIN_ID')!='174' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires the local exam simulation domain')
    output=root/'sensor-recordings';output.mkdir(exist_ok=True)
    while True:
        current=state(root)
        if current.get('state') in TERMINAL:return
        if current.get('state')!='native_episode':
            time.sleep(2);continue
        name=current['name']
        if Path(name).name!=name or not name.startswith('v151-survey-'):
            raise ValueError('Unexpected exam run')
        bag=output/name
        if bag.exists() or (recordings/name/'metrics.json').exists():
            time.sleep(2);continue
        with (output/(name+'.log')).open('a') as log:
            recorder=subprocess.Popen(['ros2','bag','record','--disable-keyboard-controls','--use-sim-time','--include-hidden-topics',
                '-s','mcap','--max-cache-size','8388608','-o',str(bag),'--topics',*TOPICS],stdout=log,stderr=log)
            try:
                while recorder.poll() is None and not (recordings/name/'metrics.json').exists():
                    status=state(root)
                    if status.get('state') in TERMINAL or status.get('name')!=name:break
                    time.sleep(2)
            finally:
                if recorder.poll() is None:
                    # A detached shell may pass an ignored SIGINT to ros2. The
                    # Jazzy record verb installs its own graceful SIGTERM handler.
                    recorder.terminate()
                    recorder.wait(timeout=30)
            if not (bag/'metadata.yaml').is_file():
                raise RuntimeError('No finalized sensor recording for '+name)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('exam',type=Path);cli.add_argument('recordings',type=Path)
    args=cli.parse_args();monitor(args.exam,args.recordings)
