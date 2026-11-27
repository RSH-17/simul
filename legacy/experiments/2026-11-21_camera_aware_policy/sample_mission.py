"""Read-only process sampler for Mission planning cost; never sends ROS data."""
import argparse
import json
from pathlib import Path
import time
import psutil


def sample(root,driver):
    previous={};streams={};last=time.monotonic()
    try:
        while not (root/'complete.json').exists():
            if not psutil.pid_exists(driver) or psutil.Process(driver).status()==psutil.STATUS_ZOMBIE:break
            now=time.monotonic();dt=now-last;last=now
            for process in psutil.process_iter(['pid','cmdline']):
                try:
                    command=' '.join(process.info['cmdline'] or [])
                    if '/lib/osp_mission/mission' not in command:continue
                    environment=process.environ()
                    if environment.get('ROS_DOMAIN_ID')!='174' or not environment.get('BENCH_RUN'):continue
                    run=Path(environment['BENCH_RUN'])
                    if not run.name.startswith('v15-'):continue
                    identity=(process.pid,process.create_time());cpu=process.cpu_times();total=cpu.user+cpu.system
                    old=previous.get(identity);previous[identity]=total
                    if old is None:continue
                    stream=streams.setdefault(str(run),None)
                    if stream is None:
                        stream=(run/'mission_resources.jsonl').open('a',buffering=1);streams[str(run)]=stream
                    stream.write(json.dumps(dict(wall_epoch_s=time.time(),cpu_percent_one_core=100*(total-old)/max(dt,.01),
                        rss_mib=process.memory_info().rss/2**20,pid=process.pid))+'\n')
                except (psutil.Error,OSError):continue
            time.sleep(1.)
    finally:
        for stream in streams.values():
            if stream is not None:stream.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('driver',type=int)
    args=parser.parse_args();sample(args.root,args.driver)
