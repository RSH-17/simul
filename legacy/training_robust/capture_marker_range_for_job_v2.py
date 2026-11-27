"""Start passive capture after the actual native process and sensors are ready.

Never launches/restarts a benchmark, changes its inputs, or crosses episodes.
The bounded watcher prevents late manual attachments missing early markers.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from observe_spatial_campaign import native_pid


def select_job(state,number):
    matches=[j for j in state['jobs'] if j['number']==number]
    if len(matches)!=1:raise ValueError('Require exactly one named prepared job')
    job=matches[0]
    if job['status']=='done':return job,None
    if job['status']!='running':return job,None
    domain=job.get('domain')
    if type(domain) is not int or not 0<=domain<=232:raise ValueError('Invalid active ROS domain')
    pid=native_pid(job['unit'],Path(job['output']))
    if pid is None:return job,None
    episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    try:
        live=json.loads((episode/'live_status.json').read_text())
        platform=json.loads((episode/'tb4_platform_observation.json').read_text())
        startup=json.loads((episode/'startup_guard.json').read_text())
    except (FileNotFoundError,json.JSONDecodeError):return job,None
    if (live.get('status',{}).get('state')!='running' or platform.get('passed') is not True
            or startup.get('success') is not True):return job,None
    return job,pid


def run(campaign,number,output,wait_seconds,capture_seconds):
    script=Path(__file__).with_name('capture_marker_range_inputs.py')
    digest=hashlib.sha256(script.read_bytes()).hexdigest();started=time.monotonic()
    launch=output.with_suffix('.launch.json')
    if output.exists() or launch.exists():raise FileExistsError('Preserve the previous capture')
    while time.monotonic()-started<wait_seconds:
        state=json.loads((campaign/'state.json').read_text());job,pid=select_job(state,number)
        if job['status']=='done':print('Episode already completed; no observer started');return
        if pid is not None:
            if hashlib.sha256(script.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Prepared capture source changed while waiting')
            stop=Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'evaluation.json'
            if stop.exists():print('Evaluation already finalized; no observer started');return
            command=[sys.executable,str(script),'--domain',str(job['domain']),
                     '--output',str(output),'--stop-file',str(stop),
                     '--seconds',str(capture_seconds),'--max-frames','200']
            launch.write_text(json.dumps(dict(job=number,observed_native_pid=pid,
                domain=job['domain'],workspace=job['workspace'],episode_stop_file=str(stop),
                capture_source_sha256=digest,command=command,read_only=True,
                benchmark_launched_or_restarted=False),indent=2)+'\n')
            subprocess.run(command,check=True)
            return
        time.sleep(3.)
    print('Bounded wait ended; no benchmark was launched or restarted')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('campaign','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--job',type=int,required=True)
    parser.add_argument('--wait-seconds',type=float,default=1800.)
    parser.add_argument('--capture-seconds',type=float,default=7200.)
    args=parser.parse_args()
    run(args.campaign.resolve(),args.job,args.output.resolve(),args.wait_seconds,args.capture_seconds)
