"""One finite localhost-only R2/R3 validation campaign; never start a physical robot."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from summarize_blind import summarize

HERE=Path(__file__).resolve().parent
HOME=Path.home()
JOB=HOME/'v151-independent-supervisor1122'
R2=HOME/'v151-adaptive-survey-paired-1122'
R3=HOME/'v151-object-followthrough-screen-112203'
QUEUE=HOME/'v151-object-followthrough-queue1122.json'
BASE=HOME/'flow_v151_exposure_sync_r2_ws'
BLIND={'R2':HOME/'v151-independent-validation-1122','R3':HOME/'v151-independent-validation-r3-112203'}
WS={'R2':HOME/'v151_adaptive_survey1122_r2_ws','R3':HOME/'v151_object_followthrough1122_r3_ws'}


def read(p):return json.loads(p.read_text())


def write(p,data):
    tmp=p.with_name(p.name+'.tmp');tmp.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(p)


def now():return datetime.now(timezone.utc).isoformat()


def observe():
    if not QUEUE.exists():return 'waiting_for_engineering_screen',None
    q=read(QUEUE)
    gate=q.get('r2_gate') or {}
    if gate.get('eligible') is True and q.get('state')=='skipped_r2_eligible':
        return 'ready','R2'
    if gate.get('eligible') is False:
        if (R3/'futility_stop.json').exists():return 'finished_r3_futility',read(R3/'futility_stop.json')
        if (R3/'candidate/complete.json').exists() and (R3/'seven_case_gate.json').exists():
            gate3=read(R3/'seven_case_gate.json')
            if gate3.get('eligible') is True:return 'ready','R3'
            if gate3.get('eligible') is False:return 'finished_both_screens_declined',gate3
        return 'waiting_for_r3_screen',q.get('state')
    return 'waiting_for_r2_export_and_gate',q.get('state')


def command(*arguments):
    return subprocess.run(['bash',str(HERE/'blind.sh'),*map(str,arguments)],capture_output=True,text=True)


def disk_budget(count):
    archive=HOME/'v151-track-five-boxes5-1795247134501974510'
    paths=read(archive/'runs.json')
    # Measured on complete original runs, never follow their shared world symlinks.
    peaks=[int(subprocess.check_output(['du','-sk',p],text=True).split()[0])*1024 for p in paths]
    return int(count*max(peaks)*1.30+3*1024**3)


def run_inputs(revision):
    suite=BLIND[revision];screen=R2 if revision=='R2' else R3
    if not (suite/'manifest.json').exists():
        if revision=='R2':raise RuntimeError('Missing pre-enrolled R2 suite')
        result=command('prepare',suite,'--screen',screen,'--baseline',BASE,'--candidate',WS[revision])
        if result.returncode:raise RuntimeError(result.stderr[-1800:])
    manifest=read(suite/'manifest.json')
    if manifest['source_screen']!=str(screen) or manifest['policies']['candidate']['workspace']!=str(WS[revision]):
        raise RuntimeError('Enrollment targets changed')
    if revision=='R3':
        previous=read(BLIND['R2']/'manifest.json')
        if previous['nonce']==manifest['nonce'] or set(previous['seeds']) & set(manifest['seeds']):
            raise RuntimeError('R3 requires a different untouched suite')
    gate=command('gate',suite)
    if gate.returncode:raise RuntimeError(gate.stderr[-1800:])
    decision=json.loads(gate.stdout.strip().splitlines()[-1])
    if not decision['eligible']:raise RuntimeError('Independent source/metric screen declined: '+str(decision['reasons']))
    return suite,manifest


def export(suite,revision):
    target=JOB/'results'/revision
    target.mkdir(parents=True,exist_ok=True)
    for file in ('manifest.json','episodes.json','result.json','routing-amendment.json'):
        src=suite/file
        if src.is_file():write(target/file,read(src))
    if (target/'result.json').exists():write(target/'topologies.json',summarize(read(target/'result.json')))
    index=[]
    rows=read(suite/'episodes.json') if (suite/'episodes.json').exists() else []
    for item in rows:
        if not item['valid']:continue
        run=HOME/'corridor-board-73bbb05/recordings'/item['run']
        audit=read(run/'audit.json');clip=run/'mission.mp4'
        with clip.open('rb') as stream:digest=hashlib.file_digest(stream,'sha256').hexdigest()
        if digest!=audit['video']['sha256']:raise RuntimeError('Completed simulation RGB hash mismatch')
        index.append(dict(role=item['role'],pair=item['pair'],run=item['run'],path=str(clip),sha256=digest,
                          bytes=clip.stat().st_size,frames=audit['video']['frames'],duration_s=audit['video']['duration_s'],
                          strict=item['strict'],registered_ids=item['registered_ids'],bumps=item['bumps']))
    write(target/'video_manifest.json',index)
    return target


def monitor(once=False):
    JOB.mkdir(exist_ok=True)
    with (JOB/'supervisor.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        final=JOB/'finished.json';status=JOB/'status.json'
        if final.exists():print(final.read_text());return
        enroll=JOB/'started.json'
        if not enroll.exists():write(enroll,dict(started_utc=now(),started_epoch_s=time.time()))
        deadline=read(enroll)['started_epoch_s']+48*3600
        def say(state,**details):
            previous=read(status) if status.exists() else {}
            row=dict(state=state,updated_utc=now(),**details);write(status,row)
            if previous.get('state')!=state or previous.get('revision')!=row.get('revision'):
                with (JOB/'transitions.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
                print(state,flush=True)
            return row
        while True:
            state,data=observe()
            if state.startswith('finished_'):
                write(final,say(state,reason=data));return
            if time.time()>deadline:
                write(final,say('finished_external_wait_expired',previous=state));return
            if state!='ready':
                say(state,engineering_queue_state=data)
                if once:return
                time.sleep(30);continue
            revision=data
            if once:say('eligible_observe_only',revision=revision);return
            suite,manifest=run_inputs(revision)
            rows_path=suite/'episodes.json'
            rows=read(rows_path) if rows_path.exists() else []
            expected=2*len(manifest['pairs'])
            if len(rows)==expected:
                dest=export(suite,revision);result=read(dest/'result.json')
                write(final,say('finished_blind_validation',revision=revision,result_dir=str(dest),
                    complete_valid=result['complete_valid'],qualified=result['task_qualification'],
                    inference=result['primary_paired_exact']));return
            free=shutil.disk_usage(suite).free;required=disk_budget(expected-len(rows))
            if free<required:
                say('paused_for_disk',revision=revision,available_bytes=free,required_bytes=required)
                time.sleep(45);continue
            if len(rows)>=2 and not rows[-1]['valid'] and not rows[-2]['valid']:
                dest=export(suite,revision);write(final,say('finished_infrastructure_hold',revision=revision,
                    reason='Two consecutive invalid/unmeasured slots, no rerun',result_dir=str(dest)));return
            say('running_blind',revision=revision,suite=str(suite),completed_before_launch=len(rows),
                pairs=9,native_launches=expected,free_bytes=free,required_bytes=required,results=str(suite/'result.json'))
            proc=command('run',suite)
            with (JOB/'runtime.log').open('a') as f:f.write(proc.stdout+proc.stderr+'\n')
            if rows_path.exists():export(suite,revision)
            updated=read(rows_path) if rows_path.exists() else []
            if proc.returncode and len(updated)==len(rows):
                if ('Resource temporarily unavailable' in proc.stderr or 'Another native benchmark' in proc.stderr
                        or 'Under 3 GiB free' in proc.stderr):
                    say('waiting_for_resource',revision=revision,reason=proc.stderr[-450:])
                    time.sleep(45);continue
                raise RuntimeError(proc.stderr[-1600:])
            if not proc.returncode and len(updated)==len(rows):
                raise RuntimeError('Validator returned without advancing eligible trial or producing a result')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--observe-once',action='store_true')
    a=p.parse_args()
    try:monitor(a.observe_once)
    except BlockingIOError:raise SystemExit('A supervisor already owns this one job')
    except Exception as exc:
        JOB.mkdir(exist_ok=True)
        write(JOB/'finished.json',dict(state='finished_integrity_or_infrastructure_hold',error=repr(exc),updated_utc=now()))
        raise
