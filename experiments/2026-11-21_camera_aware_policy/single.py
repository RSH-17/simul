"""Run one installed version on an existing frozen simulation map."""
import argparse
import fcntl
from pathlib import Path
import time
from campaign import execute,PROFILES


def run(version,seed):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    try:
        for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Comparison is already using the simulator; wait until it finishes.')
    policy,profile=version.rsplit('-',1)
    variant='baseline' if policy=='v1.2' else 'candidate'
    root=Path.home()/'camera-aware-policy-1121'
    assert (root/f'map-{seed}/world/osp_studio.sdf').is_file()
    execute(root,variant,profile,seed,'manual-'+str(time.time_ns()),300)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('version',choices=[p+'-'+profile for p in ('v1.2','v1.5.0') for profile in PROFILES])
    parser.add_argument('seed',type=int,choices=(112021,112022,112101,112102,112103))
    args=parser.parse_args();run(args.version,args.seed)
