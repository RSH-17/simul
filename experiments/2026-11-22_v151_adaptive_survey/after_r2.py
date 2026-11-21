"""One-shot chain: verified R2 export -> optional R3 engineering screen, no blind or robot."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'validation'))
from decisions import screen
from compare import report


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def store(path,state):
    tmp=path.with_suffix('.new')
    tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(path)


def verified_export(dst):
    if not (dst/'comparison.json').is_file() or not read(dst/'comparison.json').get('complete'):
        return False
    archive=dst.with_suffix('.tgz')
    if not archive.is_file():
        return False
    bundle=read(dst/'media.json')
    if len(bundle)!=9 or len({row['run'] for row in bundle})!=9:
        raise ValueError('Expected seven candidate and two new baseline recordings')
    if any(not (dst/(split+'-routes.png')).is_file() for split in ('regression','heldout')):
        raise ValueError('Final route figure missing')
    # gzip emit/close can race the watcher. Demand a readable, complete tar.
    try:
        with tarfile.open(archive,'r:gz') as compressed:
            members=set(compressed.getnames())
    except (EOFError,OSError,tarfile.ReadError):
        return False
    for row in bundle:
        path=dst/'training_robust/native90/v151-adaptive-survey-1122'/row['run']/'video/mission.mp4'
        if str(path.relative_to(dst.parent)) not in members or digest(path)!=row['sha256']:
            raise ValueError('Synthetic recording did not verify: '+row['run'])
    return True


def chain(args):
    parent=args.parent.resolve();root=args.root.resolve();workspace=args.workspace.resolve()
    status=args.status.resolve();deadline=time.monotonic()+args.hours*3600
    info=dict(parent=str(parent),root=str(root),workspace=str(workspace),pid=os.getpid(),watchdog_hours=args.hours,
              physical_drive=False,blind_launch=False,started_at=time.time())
    with status.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if status.exists() and read(status).get('state') in ('skipped_r2_eligible','futility_stop','complete'):
            print('ALREADY TERMINAL',status,flush=True);return
        def update(state,**details):
            info.update(state=state,updated_at=time.time(),**details)
            store(status,info);print(state,json.dumps(details),flush=True)
        update('waiting_for_r2_archive')
        try:
            while time.monotonic()<deadline:
                if (parent/'candidate/complete.json').is_file() and verified_export(args.export.resolve()):
                    break
                time.sleep(30)
            else:raise TimeoutError('R2 final archive unavailable at deadline')
            current=report(parent)
            if not current['complete']:
                raise RuntimeError('Completed archive differs from current comparison')
            gate=screen(current)
            if gate['eligible']:
                update('skipped_r2_eligible',r2_gate=gate)
                return
            if digest(workspace/'followthrough_manifest.json')!=args.sha256:
                raise ValueError('Frozen R3 manifest does not match preregistered version')
            update('running_r3',r2_gate=gate)
            for action in ('prepare','run'):
                subprocess.run(['bash',str(HERE/'rescreen.sh'),action,str(parent),str(root),str(workspace)],check=True)
            if (root/'futility_stop.json').is_file():
                update('futility_stop',r3_gate=read(root/'futility_stop.json'))
            elif (root/'seven_case_gate.json').is_file():
                update('complete',r3_gate=read(root/'seven_case_gate.json'))
            else:raise RuntimeError('R3 ended without an adjudicated decision')
        except BaseException as exc:
            update('needs_investigation',error=str(exc))
            raise


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('parent',type=Path);p.add_argument('root',type=Path)
    p.add_argument('workspace',type=Path);p.add_argument('--export',type=Path,required=True)
    p.add_argument('--status',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--hours',type=float,default=10.)
    chain(p.parse_args())
