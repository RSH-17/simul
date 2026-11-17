"""Capture policy executions and publish separate simulation/robot log assets.

Raw logs, bags and maps go to Release assets; only small per-run indexes enter
Git. No credentials or full environment dump are collected. Failed uploads stay
pending locally. This module never starts a robot except via an explicit run
command, and the background sync only observes completed executions.
"""
import argparse
import base64
from datetime import datetime,timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import tarfile
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
REPO='peter010905/simul'
MODES=('simulation','robot')
GH=shutil.which('gh') or str(Path.home()/'.local/bin/gh')
SKIP={'.git','build','install','src','source_snapshot','__pycache__','.pytest_cache'}


def now():return datetime.now(timezone.utc).isoformat()
def save(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temp.replace(path)
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def command(args,timeout=120,**kwargs):
    return subprocess.run(args,check=True,capture_output=True,text=True,timeout=timeout,**kwargs).stdout
def identity(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return None if fields[0]=='Z' else fields[19]
    except (FileNotFoundError,ProcessLookupError):return None
def boot_id():return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
def alive(manifest,prefix):
    return (manifest.get('boot_id')==boot_id() and manifest.get(prefix+'_pid')
        and manifest.get(prefix+'_start') is not None and identity(manifest[prefix+'_pid'])==manifest[prefix+'_start'])
def child_group_alive(manifest):
    if manifest.get('boot_id')!=boot_id() or not manifest.get('child_pid'):return False
    try:os.killpg(manifest['child_pid'],0);return True
    except ProcessLookupError:return False
def git_revision(root):
    try:return command(['git','rev-parse','HEAD'],cwd=root).strip()
    except (OSError,subprocess.SubprocessError):return None


def bag_command(directory,mode,namespace=''):
    args=['ros2','bag','record','--storage','mcap','--storage-preset-profile','zstd_fast',
        '--max-cache-size','16777216','--max-bag-size','268435456','--include-hidden-topics',
        '--disable-keyboard-controls','--output',str(directory)]
    if namespace:
        prefix='/'+re.escape(namespace.strip('/'))
        args+=['--regex','^('+prefix+'(/|$)|/(tf|tf_static|clock|rosout|parameter_events)$)']
    else:args+=['--all']
    if mode=='robot':args+=['--exclude-regex',r'^(/[^/]+)*/oakd/(?!rgb/preview/(image_raw|camera_info)$).*']
    if mode=='simulation':args+=['--use-sim-time']
    return args


def stop(process):
    if process is None:return None
    if process.poll() is None:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
    try:return process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:return process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);return process.wait(timeout=5)


def run_policy(root,mode,argv,workspace=None,policy=None,namespace='',bag=True,upload=True,validation=False,artifacts=()):
    if mode not in MODES or not argv:raise ValueError('Explicit mode and policy command are required')
    if mode=='robot' and any(v.lower() in ['use_sim_time:=true','use_sim_time:=1'] for v in argv):raise ValueError('Sim-time launch cannot be labelled robot')
    ident=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:10]
    run=root/'run_logs'/mode/ident;run.mkdir(parents=True,exist_ok=False)
    extra=[str(Path(p).resolve()) for p in artifacts]
    if argv[:3]==['ros2','launch','osp_mission']:
        extra += [str(Path(v.split(':=',1)[1]).resolve()) for v in argv if v.startswith('output_dir:=')]
    if any(run.is_relative_to(Path(p)) for p in extra):raise ValueError('Artifact directory cannot contain the logger run directory')
    manifest=dict(schema_version=1,id=ident,mode=mode,started_utc=now(),state='recording',
        git_commit=git_revision(root),workspace=str(workspace) if workspace else None,
        logger_sha256=sha(Path(__file__)),artifact_directories=extra,
        policy_path=str(policy) if policy else None,command=argv,
        evidence_type='logging_smoke_test' if validation else 'policy_execution',
        ros_domain=os.environ.get('ROS_DOMAIN_ID','0'),namespace=namespace,
        bag_enabled=bag,auto_upload=upload,wrapper_pid=os.getpid(),wrapper_start=identity(os.getpid()),boot_id=boot_id(),
        environment_dump_collected=False)
    save(run/'run.json',manifest)
    env=dict(os.environ,OSP_LOGGED_RUN='1',OSP_RUN_DIR=str(run),ROS_LOG_DIR=str(run/'ros_logs'))
    # Direct ros2 launch users receive the same isolated artifact directory.
    child_argv=list(argv)
    if argv[:3]==['ros2','launch','osp_mission'] and not any(v.startswith('output_dir:=') for v in argv):
        child_argv+=['output_dir:='+str(run/'artifacts')]
    recorder=None;child=None;recorder_log=None;received=[];exit_code=1;failure=None
    old_handlers={}
    def forward(sig,frame):
        received.append(sig)
        if child is not None and child.poll() is None:
            try:os.killpg(child.pid,sig)
            except ProcessLookupError:pass
    for sig in [signal.SIGINT,signal.SIGTERM]:old_handlers[sig]=signal.signal(sig,forward)
    try:
        if bag:
            recorder_log=(run/'recorder.log').open('wb')
            recorder=subprocess.Popen(bag_command(run/'rosbag',mode,namespace),env=env,
                stdout=recorder_log,stderr=recorder_log,start_new_session=True)
            manifest.update(recorder_pid=recorder.pid,recorder_start=identity(recorder.pid));save(run/'run.json',manifest)
            time.sleep(.5)
            if recorder.poll() is not None:raise RuntimeError('ROS bag recorder failed before launch; inspect recorder.log')
        if received:raise InterruptedError('Stopped before policy launch')
        with (run/'console.log').open('wb') as console:
            child=subprocess.Popen(child_argv,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
            manifest.update(child_pid=child.pid,child_start=identity(child.pid));save(run/'run.json',manifest)
            while True:
                data=child.stdout.read1(65536)
                if not data:break
                console.write(data);console.flush()
                try:os.write(1,data)
                except OSError:pass
            exit_code=child.wait()
    except Exception as exc:
        failure=type(exc).__name__+': '+str(exc)
        (run/'capture_error.txt').write_text(failure+'\n')
    finally:
        if child is not None and child.poll() is None:stop(child)
        if child is not None and child.stdout:child.stdout.close()
        recorder_code=stop(recorder)
        if recorder_log:recorder_log.close()
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
        manifest.update(state='interrupted' if received else 'finished' if failure is None else 'capture_failed',
            finished_utc=now(),exit_code=exit_code,signals=received,capture_error=failure,recorder_exit=recorder_code,
            capture_complete=failure is None and (not bag or recorder_code in [0,-signal.SIGINT,130]),
            upload_state='pending' if upload else 'disabled')
        if policy and Path(policy).is_file():
            (run/'policy.json').write_bytes(Path(policy).read_bytes());manifest['policy_sha256']=sha(Path(policy))
        if workspace:
            configs=[Path(workspace)/'nav2_robot_active.yaml',
                     root/'robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml']
            configs+=list((Path(workspace)/'install/osp_mission/share/osp_mission/config').glob('*.yaml'))
            for path in configs:
                if path.is_file():
                    (run/'config').mkdir(exist_ok=True);(run/'config'/path.name).write_bytes(path.read_bytes())
        save(run/'run.json',manifest)
    print('\nRun logs: '+str(run),flush=True)
    if upload:
        try:publish(run,REPO)
        except BlockingIOError:print('Upload already owned by sync; local logs retained.',flush=True)
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
            save(run/'upload.json',dict(state='pending',last_error=str(exc),attempt_utc=now()))
            print('Upload pending; local logs preserved for retry: '+type(exc).__name__,flush=True)
    return (128+received[-1]) if received else exit_code


class Segments:
    def __init__(self,directory,name,limit):self.directory=directory;self.name=name;self.limit=limit;self.files=[];self.stream=None;self.size=0
    def write(self,data):
        total=len(data)
        while data:
            if self.stream is None or self.size==self.limit:
                if self.stream:self.stream.close()
                path=self.directory/f'{self.name}.tar.gz.part{len(self.files)+1:04d}'
                self.stream=path.open('xb');self.files.append(path);self.size=0
            count=min(len(data),self.limit-self.size);self.stream.write(data[:count]);self.size+=count;data=data[count:]
        return total
    def flush(self):
        if self.stream:self.stream.flush()
    def close(self):
        if self.stream:self.stream.close()


def archive_files(base):
    for path in sorted(base.rglob('*')):
        relative=path.relative_to(base)
        if path.is_symlink() or not path.is_file() or set(relative.parts)&SKIP:continue
        if path.suffix in ['.mp4','.pem','.key'] or path.name.startswith('.env'):continue
        yield path,relative


def package(run,manifest,limit=256*1024*1024):
    inventory=run/'archive.json'
    if inventory.exists():
        result=json.loads(inventory.read_text())
        for asset in result['assets']:
            p=run/'upload_parts'/asset['name']
            if p.stat().st_size!=asset['bytes'] or sha(p)!=asset['sha256']:raise ValueError('Frozen archive changed')
        return result
    parts=run/'upload_parts'
    if parts.exists():
        # Only our interrupted staging files are recoverable; raw logs are untouched.
        for p in parts.iterdir():
            if p.is_file() and p.name.startswith(manifest['id']+'.tar.gz.part'):p.unlink()
            else:raise ValueError('Unknown file in archive staging; preserving it')
    else:parts.mkdir()
    sink=Segments(parts,manifest['id'],limit);roots=[(run,Path('run'))]
    if manifest.get('native_output'):roots.append((Path(manifest['native_output']),Path('native')))
    roots += [(Path(p),Path('external_artifacts')/str(i)) for i,p in enumerate(manifest.get('artifact_directories',[]))]
    try:
        with tarfile.open(mode='w|gz',fileobj=sink,compresslevel=3) as archive:
            for base,prefix in roots:
                for path,relative in archive_files(base):
                    if base==run and (relative.parts[0]=='upload_parts' or path.name in ['archive.json','upload.json','upload.lock','github_index_request.json']):continue
                    archive.add(path,arcname=str(prefix/relative),recursive=False)
    finally:sink.close()
    result=dict(format='Concatenate numbered parts, then extract as tar.gz',
        assets=[dict(name=p.name,bytes=p.stat().st_size,sha256=sha(p)) for p in sink.files])
    save(inventory,result);return result


def gh_json(args):return json.loads(command([GH,*args]))


def publish(run,repo=REPO):
    with (run/'upload.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        done=run/'upload.json'
        if done.exists() and json.loads(done.read_text()).get('state')=='uploaded':return
        manifest=json.loads((run/'run.json').read_text())
        if manifest['mode'] not in MODES or manifest['state']=='recording':raise ValueError('Only terminal classified runs can be uploaded')
        if not re.fullmatch(r'[A-Za-z0-9_-]+',manifest['id']):raise ValueError('Invalid run ID')
        month=manifest['started_utc'][:7];tag='policy-logs-'+manifest['mode']+'-'+month
        try:release=gh_json(['api',f'repos/{repo}/releases/tags/{tag}'])
        except subprocess.CalledProcessError as exc:
            if 'HTTP 404' not in exc.stderr:raise
            subprocess.run([GH,'release','create',tag,'--repo',repo,'--target','main','--latest=false',
                '--title',manifest['mode']+' policy logs '+month,'--notes','Automatic policy execution logs. Raw data may include sensor observations. Runs are separated by simulation/robot; uploads do not establish task success.'],capture_output=True,text=True,timeout=120)
            release=gh_json(['api',f'repos/{repo}/releases/tags/{tag}'])
        archive=package(run,manifest)
        def assets():return {item['name']:item for page in gh_json(['api',f'repos/{repo}/releases/{release["id"]}/assets?per_page=100','--paginate','--slurp']) for item in page}
        remote=assets()
        for item in archive['assets']:
            found=remote.get(item['name'])
            if found is None:
                command([GH,'release','upload',tag,str(run/'upload_parts'/item['name']),'--repo',repo],timeout=600)
            elif found['size']!=item['bytes'] or found.get('digest')!='sha256:'+item['sha256']:
                raise ValueError('Remote asset mismatch; never overwrite an existing asset')
        remote=assets()
        for item in archive['assets']:
            found=remote.get(item['name'])
            if not found or found['size']!=item['bytes'] or found.get('digest')!='sha256:'+item['sha256']:
                raise ValueError('Remote log archive hash verification failed')
            item['url']=found['browser_download_url']
        index=dict(schema_version=1,id=manifest['id'],mode=manifest['mode'],state=manifest['state'],
            evidence_type=manifest['evidence_type'],started_utc=manifest['started_utc'],finished_utc=manifest.get('finished_utc'),
            git_commit=manifest.get('git_commit'),policy_sha256=manifest.get('policy_sha256'),exit_code=manifest.get('exit_code'),
            capture_complete=manifest.get('capture_complete'),bag_enabled=manifest.get('bag_enabled'),
            release_url=release['html_url'],archive=archive,uploaded_utc=now(),task_success_claimed=False)
        relative=f'repository_snapshot/run_logs/{manifest["mode"]}/{manifest["id"]}.json'
        payload=dict(message='Index '+manifest['mode']+' policy run '+manifest['id'],branch='main',
            content=base64.b64encode((json.dumps(index,ensure_ascii=False,indent=2)+'\n').encode()).decode())
        try:
            old=gh_json(['api',f'repos/{repo}/contents/{relative}?ref=main'])
            previous=json.loads(base64.b64decode(old['content']))
            if previous['archive']!=index['archive']:raise ValueError('Existing run index differs; preserving it')
        except subprocess.CalledProcessError as exc:
            if 'HTTP 404' not in exc.stderr:raise
            temp=run/'github_index_request.json';temp.write_text(json.dumps(payload))
            try:gh_json(['api',f'repos/{repo}/contents/{relative}','--method','PUT','--input',str(temp)])
            finally:temp.unlink(missing_ok=True)
        save(done,dict(state='uploaded',verified_utc=now(),release_url=release['html_url'],index=relative,assets=archive['assets']))
        print('Uploaded '+manifest['mode']+' logs: '+release['html_url'],flush=True)


def discover_native(root,since):
    """Read finalized native files; never mutate frozen experiments or launch jobs."""
    for manifest_path in sorted((root/'training_robust/native90').glob('*/jobs/*/manifest.json')):
        try:
            native=manifest_path.parent;manifest=json.loads(manifest_path.read_text())
            if manifest.get('environment')!='gazebo_harmonic':continue
            report=native/'report.json'
            final_file=report if report.exists() else native/'service.log'
            if not final_file.exists() or final_file.stat().st_mtime<since:continue
            finalized=report.exists() and len(json.loads(report.read_text()).get('rows',[]))==1
            unit_file=native/'service.json'
            if unit_file.exists():
                unit=json.loads(unit_file.read_text())['unit']
                state=subprocess.run(['systemctl','--user','show',unit,'-p','LoadState','-p','ActiveState','-p','MainPID'],capture_output=True,text=True,timeout=10)
                fields=dict(v.split('=',1) for v in state.stdout.splitlines() if '=' in v)
                if not fields:continue
                if state.returncode and fields.get('LoadState')!='not-found':continue
                if fields.get('ActiveState') in ['active','activating','deactivating']:continue
            elif not finalized:continue
            # Also verify no benchmark process still owns this directory.
            busy=False
            for proc in Path('/proc').glob('[0-9]*/cmdline'):
                try:
                    argv=proc.read_bytes().split(b'\0')
                    if any(v.endswith(b'/gazebo_benchmark.py') for v in argv) and str(native).encode() in argv:busy=True;break
                except (OSError,ProcessLookupError):pass
            if busy:continue
            ident='native-'+hashlib.sha256(str(native).encode()).hexdigest()[:20]
            run=root/'run_logs/simulation'/ident
            if (run/'run.json').exists():continue
            run.mkdir(parents=True,exist_ok=False)
            policy=native/'policy.json'
            save(run/'run.json',dict(schema_version=1,id=ident,mode='simulation',state='finished' if finalized else 'capture_failed',
                evidence_type='native_gazebo_policy_execution',started_utc=datetime.fromtimestamp(manifest_path.stat().st_mtime,timezone.utc).isoformat(),
                finished_utc=datetime.fromtimestamp(final_file.stat().st_mtime,timezone.utc).isoformat(),
                native_output=str(native),policy_sha256=sha(policy) if policy.exists() else None,
                git_commit=None,exit_code=None,bag_enabled=False,capture_complete=finalized,auto_upload=True,
                recording_scope='Existing native artifacts and ROS logs; no retroactive rosbag or live capture is claimed. MP4 uses the separate video publisher.'))
        except (OSError,ValueError,KeyError,subprocess.SubprocessError) as exc:
            print('Native log discovery deferred: '+str(manifest_path)+' '+type(exc).__name__,flush=True)


def sync(root,repo,limit=3,since=None):
    if limit<1:raise ValueError('limit must be positive')
    logs=root/'run_logs';logs.mkdir(exist_ok=True)
    with (logs/'sync.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        config=logs/'sync.json'
        if not config.exists():save(config,dict(enabled_unix=time.time(),enabled_utc=now()))
        discover_native(root,since if since is not None else json.loads(config.read_text())['enabled_unix'])
        attempts=0
        for path in sorted(logs.glob('*/*/run.json')):
            try:manifest=json.loads(path.read_text())
            except (OSError,ValueError):
                print('Invalid run manifest, preserving for inspection: '+str(path),flush=True);continue
            if manifest.get('state')=='recording':
                if alive(manifest,'wrapper') or alive(manifest,'child') or child_group_alive(manifest):continue
                # A killed wrapper must never silently turn into a successful run.
                # Do not signal a remaining policy/robot process; defer while alive.
                if alive(manifest,'recorder'):
                    os.killpg(manifest['recorder_pid'],signal.SIGINT)
                    continue
                manifest.update(state='interrupted_unclean',finished_utc=now(),capture_complete=False,
                    exit_code=None,capture_error='Wrapper disappeared; recovered by sync. Bag may need reindexing.')
                save(path,manifest)
            if not manifest.get('auto_upload'):continue
            upload=path.parent/'upload.json'
            if upload.exists() and json.loads(upload.read_text()).get('state')=='uploaded':continue
            try:publish(path.parent,repo)
            except BlockingIOError:continue
            except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
                save(path.parent/'upload.json',dict(state='pending',last_error=str(exc),attempt_utc=now()))
                print('Pending upload: '+manifest['id']+' '+type(exc).__name__,flush=True)
            attempts+=1
            if attempts>=limit:break


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='action',required=True)
    run=sub.add_parser('run');run.add_argument('--mode',choices=MODES,required=True)
    run.add_argument('--workspace',type=Path);run.add_argument('--policy',type=Path);run.add_argument('--namespace',default='')
    run.add_argument('--artifact',type=Path,action='append',default=[],help='Additional explicit output directory to archive')
    run.add_argument('--no-bag',action='store_true');run.add_argument('--no-upload',action='store_true')
    run.add_argument('--validation-only',action='store_true');run.add_argument('command',nargs=argparse.REMAINDER)
    up=sub.add_parser('upload');up.add_argument('run',type=Path)
    scan=sub.add_parser('sync');scan.add_argument('--limit',type=int,default=3);scan.add_argument('--since',type=float)
    for p in [run,up,scan]:p.add_argument('--repo',default=REPO);p.add_argument('--root',type=Path,default=ROOT)
    a=parser.parse_args()
    if a.action=='upload':publish(a.run.resolve(),a.repo)
    elif a.action=='sync':sync(a.root.resolve(),a.repo,a.limit,a.since)
    else:
        argv=a.command[1:] if a.command[:1]==['--'] else a.command
        if a.repo!=REPO:parser.error('Use upload/sync for a different repository')
        raise SystemExit(run_policy(a.root.resolve(),a.mode,argv,a.workspace,a.policy,a.namespace,
            not a.no_bag,not a.no_upload,a.validation_only,a.artifact))


if __name__=='__main__':main()
