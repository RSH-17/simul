"""Capture full-rate wheel/IMU inputs from one verified existing native job.

Starts only a read-only recorder, never a benchmark. The original benchmark
process identity bounds collection so reused ROS domains cannot mix episodes.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from capture_marker_range_for_job_v2 import select_job


def identity(pid):
    try:return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except FileNotFoundError:return None


def stop_reason(native_identity,observed_identity,evaluation_exists,elapsed,limit):
    if evaluation_exists:return 'episode_completed'
    if observed_identity!=native_identity:return 'original_native_process_ended'
    if elapsed>=limit:return 'bounded_recording_window'
    return None


def save(path,payload):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(payload,indent=2)+'\n');temporary.replace(path)


def run(campaign,number,output,wait_seconds,capture_seconds):
    source=Path(__file__).resolve();recorder=source.with_name('record_odometry_inputs.py')
    sources=[source,recorder,source.with_name('capture_marker_range_for_job_v2.py'),source.with_name('observe_spatial_campaign.py')]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    output.mkdir(parents=True,exist_ok=False);save(output/'source_sha256.json',hashes)
    started=time.monotonic();job=None;pid=None
    while time.monotonic()-started<wait_seconds:
        for name,expected in hashes.items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:raise ValueError('Queued recorder source changed')
        job,pid=select_job(json.loads((campaign/'state.json').read_text()),number)
        if job['status']=='done':
            save(output/'report.json',dict(status='not_started',reason='episode_already_complete'));return
        if pid is not None:break
        save(output/'progress.json',dict(status='waiting_for_verified_native_process',job=number))
        time.sleep(2.)
    else:
        save(output/'report.json',dict(status='not_started',reason='bounded_wait'));return
    original=identity(pid)
    if original is None:raise RuntimeError('Native process ended before recorder attached')
    episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    command=[sys.executable,str(recorder),'--domain',str(job['domain']),
             '--seconds',str(capture_seconds),'--output',str(output/'inputs.csv')]
    metadata=dict(job=number,episode=str(episode),domain=job['domain'],native_pid=pid,
        native_process_start=original,command=command,ground_truth_input=False,
        controls_robot=False,tf_published=False,benchmark_launched_or_restarted=False,
        sensor_scope='Odometry pose/twist and IMU quaternion/yaw rate; absolute IMU orientation is not fused')
    save(output/'launch.json',metadata);platform_checks=0;reason=None;started=time.monotonic()
    with (output/'recorder.log').open('x') as log:
        child=subprocess.Popen(command,stdout=log,stderr=log)
        try:
            while child.poll() is None:
                reason=stop_reason(original,identity(pid),(episode/'evaluation.json').exists(),
                                   time.monotonic()-started,capture_seconds)
                if reason:break
                try:
                    platform=json.loads((episode/'tb4_platform_observation.json').read_text())
                    platform_checks+=1
                    if platform.get('passed') is not True:reason='platform_guard_failure';break
                except (FileNotFoundError,json.JSONDecodeError):pass
                save(output/'progress.json',dict(status='recording',recorder_pid=child.pid,
                    native_pid=pid,platform_checks=platform_checks,wall_seconds=time.monotonic()-started))
                time.sleep(2.)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:child.kill();child.wait();reason='recorder_shutdown_timeout'
    counts={}
    if reason is None and child.returncode==0 and time.monotonic()-started>=capture_seconds:
        reason='bounded_recording_window'
    path=output/'inputs.csv'
    if path.exists():
        with path.open() as stream:
            for row in csv.DictReader(stream):counts[row['sensor']]=counts.get(row['sensor'],0)+1
    save(output/'report.json',dict(metadata,status='finished',stop_reason=reason or 'recorder_returned',
        recorder_returncode=child.returncode,wall_seconds=time.monotonic()-started,
        platform_checks=platform_checks,counts=counts,input_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None,
        complete=reason in ['episode_completed','original_native_process_ended','bounded_recording_window'] and child.returncode in [0,-signal.SIGINT,130] and all(counts.get(k,0)>1 for k in ['odom','imu'])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['campaign','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--job',type=int,required=True);p.add_argument('--wait-seconds',type=float,default=3600.)
    p.add_argument('--capture-seconds',type=float,default=7200.);a=p.parse_args()
    run(a.campaign.resolve(),a.job,a.output.resolve(),a.wait_seconds,a.capture_seconds)
