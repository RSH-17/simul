"""One finite, explicit dispatcher for an already enrolled exam-track comparison."""
# This directory also contains the ROS launch entrypoint. Its filename queue.py
# must not shadow Python's queue module when standard-library clients import it.
if __name__ == 'queue':
    import importlib.util
    import sys
    import sysconfig
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('queue',Path(sysconfig.get_path('stdlib'))/'queue.py')
    module=importlib.util.module_from_spec(spec)
    sys.modules['queue']=module
    spec.loader.exec_module(module)
else:
    import argparse
    from contextlib import ExitStack
    from datetime import datetime, timezone
    import fcntl
    import hashlib
    import json
    import os
    from pathlib import Path
    import subprocess
    import sys
    import time

    from select_finalist import R4_TOOLS, read, select, stable

    HERE=Path(__file__).resolve().parent

    def write(path,obj):
        temp=path.with_name(path.name+'.new')
        temp.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
        temp.replace(path)

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def utc():
        return datetime.now(timezone.utc).isoformat()

    def locks_free():
        with ExitStack() as scope:
            for name in ('.domain.lock','.second-domain.lock'):
                fd=scope.enter_context((Path.home()/'track-recordings'/name).open('a'))
                try:
                    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    return False
        return True

    def verify_selection_code(protocol):
        files=(HERE/'select_finalist.py',HERE/'queue.py',R4_TOOLS/'track_screen.py')
        if protocol.get('selection_files')!={p.name:digest(p) for p in files}:
            raise ValueError('Enrolled selection code differs from active code')

    def finished(root,decision):
        folder=root/decision['revision']
        if not (folder/'result.json').exists():
            return None
        result=read(folder/'result.json')
        protocol=read(root/'protocol.json'); revision=decision['revision']
        expected=dict(protocol_sha256=stable(protocol),revision=revision,
                      baseline_source=protocol['policies']['baseline']['source_manifest_sha256'],
                      candidate_source=protocol['policies'][revision]['source_manifest_sha256'])
        if read(folder/'enrollment.json')!=expected or result['revision']!=revision:
            raise ValueError('Native result belongs to a different enrollment or policy')
        n=decision['required_pairs']
        if result.get('recorded_episodes')!=2*n:
            return None
        ok=(result.get('completed_pairs')==result.get('valid_pairs')==n and
            (result.get('complete_valid') is True if n==9 else result.get('valid_anchor_repetitions')==2))
        return dict(state='completed_'+decision['mode'] if ok else 'infrastructure_hold',required_pairs=n,
                    valid_pairs=result.get('valid_pairs'),recorded_episodes=result['recorded_episodes'],
                    development_qualified=decision['development_qualified'],result=str(folder/'result.json'),
                    automatic_promotion=False)

    def run_queue(root,hours,poll):
        job=root/'official-queue'
        job.mkdir(exist_ok=True,parents=True)
        with (job/'owner.lock').open('a') as owner:
            fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
            terminal=job/'terminal.json';status=job/'status.json';selection=job/'selection.json'
            dispatch=job/'dispatch.json';start=job/'start.json'
            if terminal.exists():
                return read(terminal)
            if not start.exists():
                write(start,dict(utc=utc(),deadline_unix=time.time()+hours*3600))
            deadline=read(start)['deadline_unix']
            def say(state,**detail):
                value=dict(state=state,utc=utc(),**detail)
                last=read(status) if status.exists() else {}
                write(status,value)
                if (last.get('state'),last.get('reason'))!=(state,detail.get('reason')):
                    print(json.dumps(value,ensure_ascii=False),flush=True)
                return value
            def close(state,**detail):
                data=say(state,**detail)
                write(terminal,data)
                return data
            def pause(state,reason):
                if time.time()>=deadline:
                    return close('infrastructure_hold',reason='Finite wait expired: '+reason)
                say(state,reason=reason)
                time.sleep(min(poll,max(0,deadline-time.time())))
            try:
                while True:
                    if not (root/'protocol.json').is_file():
                        done=pause('waiting_for_exam_enrollment','Official protocol is absent')
                        if done:return done
                        continue
                    verify_selection_code(read(root/'protocol.json'))
                    if selection.exists():
                        decision=read(selection)
                        if (decision['protocol_sha256']!=digest(root/'protocol.json') or
                                decision['selector_sha256']!=digest(HERE/'select_finalist.py') or
                                decision['dispatcher_sha256']!=digest(HERE/'queue.py')):
                            return close('infrastructure_hold',reason='The pinned exam or selection implementation changed')
                    else:
                        decision=select(root)
                        if decision['mode']=='wait':
                            done=pause('waiting_for_development',decision['reason'])
                            if done:return done
                            continue
                        if decision['mode']=='hold':
                            return close('infrastructure_hold',reason=decision['reason'])
                        decision.update(selector_sha256=digest(HERE/'select_finalist.py'),dispatcher_sha256=digest(HERE/'queue.py'))
                        write(selection,decision)
                    prior=finished(root,decision)
                    if prior:return close(**prior)
                    if dispatch.exists():
                        return close('infrastructure_hold',reason='Recorded dispatch did not finish; preserve partial episodes, no rerun')
                    if not locks_free():
                        done=pause('waiting_for_native_engine','Native simulation locks are held')
                        if done:return done
                        continue
                    revision=decision['revision']
                    cmd=[sys.executable,'-Bu',str(HERE/'paired.py'),'run',str(root),'--revision',revision]
                    if decision['mode']=='anchor_only':cmd.append('--anchor-only')
                    say('dispatching',revision=revision,mode=decision['mode'],required_pairs=decision['required_pairs'])
                    write(dispatch,dict(utc=utc(),revision=revision,mode=decision['mode'],pid=os.getpid()))
                    # paired.py takes both native locks atomically, validates the enrollment,
                    # applies its per-episode watchdog and records a write-once attempt.
                    with (job/'paired.log').open('ab') as log:
                        proc=subprocess.run(cmd,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
                    completion=finished(root,decision)
                    if proc.returncode or not completion:
                        return close('infrastructure_hold',revision=revision,returncode=proc.returncode,
                                    reason='Runner exited with missing or invalid evidence. No performance rerun.')
                    return close(**completion)
            except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
                return close('infrastructure_hold',reason=repr(exc))

    def main():
        args=argparse.ArgumentParser(description=__doc__)
        args.add_argument('root',type=Path)
        args.add_argument('--hours',type=float,default=36)
        args.add_argument('--poll',type=float,default=30)
        a=args.parse_args()
        if not 0<a.hours<=72 or not 1<=a.poll<=60:
            args.error('Use 0 < hours <= 72 and 1 <= poll <= 60 seconds')
        try:
            result=run_queue(a.root.resolve(),a.hours,a.poll)
        except BlockingIOError:
            print('Another dispatcher owns this official job',flush=True)
            return 75
        print(json.dumps(result,ensure_ascii=False),flush=True)
        return 0 if result['state'].startswith('completed_') else 75

    if __name__=='__main__':
        sys.exit(main())
