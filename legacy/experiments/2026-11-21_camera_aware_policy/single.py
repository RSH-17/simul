"""Run one installed version on an existing frozen simulation map."""
import argparse
import fcntl
from pathlib import Path
import time
import psutil
from campaign import execute,PROFILES,WORKSPACES


def run(version,seed):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    try:
        for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Comparison is already using the simulator; wait until it finishes.')
    for process in psutil.process_iter():
        try:
            env=process.environ()
            if env.get('ROS_DOMAIN_ID')=='174' and env.get('BENCH_RUN'):
                raise SystemExit('An existing simulation still owns domain 174; inspect its state before restarting.')
        except psutil.Error:pass
    policy,profile=version.rsplit('-',1)
    variant={'v1.2':'baseline','v1.5.0':'candidate','v1.5.1':'sync','v1.6.0':'committed'}[policy]
    if variant=='sync':WORKSPACES[variant]=Path.home()/'flow_v151_exposure_sync_r2_ws'
    if variant=='committed':WORKSPACES[variant]=Path.home()/'flow_v16_local_commitment_r3_ws'
    root=Path.home()/'camera-aware-policy-1121'
    if seed>=112201:root=Path.home()/'camera-refine-policy-1121'
    if seed>=112301:root=Path.home()/'camera-final-policy-1121'
    assert (root/f'map-{seed}/world/osp_studio.sdf').is_file()
    execute(root,variant,profile,seed,'manual-'+str(time.time_ns()),300)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('version',choices=[p+'-'+profile for p in ('v1.2','v1.5.0','v1.5.1') for profile in PROFILES]+['v1.6.0-detail'])
    parser.add_argument('seed',type=int,choices=(112021,112022,112101,112102,112103,112201,112202,112203,112301,112302,112303))
    args=parser.parse_args();run(args.version,args.seed)
