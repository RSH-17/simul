"""Sequential isolated capture, replay screening, then native candidate trials."""
import argparse,fcntl,hashlib,json,os,subprocess,sys,time
from pathlib import Path
import psutil,yaml

HERE=Path(__file__).resolve().parent


def cleanup(run):
    owned=[]
    for p in psutil.process_iter(['pid']):
        try:
            e=p.environ()
            if e.get('BENCH_RUN')==str(run) and e.get('ROS_DOMAIN_ID')=='174':owned.append(p)
        except psutil.Error:pass
    for p in owned:
        try:p.terminate()
        except psutil.Error:pass
    _,alive=psutil.wait_procs(owned,timeout=3)
    for p in alive:
        try:p.kill()
        except psutil.Error:pass


def native(root,map_id,size,fps,tag,duration,lossless=False):
    fixture=root/f'map-{map_id}'
    run=Path.home()/'corridor-board-73bbb05/recordings'/f'v12-{tag}-{size}px-{fps}fps-m{map_id}-expanded1120'
    stem=run.name;attempt=1
    while run.exists():
        attempt+=1;run=run.with_name(stem+f'-retry{attempt}')
    run.mkdir(exist_ok=False)
    cfg=yaml.safe_load((fixture/'scene.yaml').read_text());cfg['duration']=float(duration)
    (run/'scene.yaml').write_text(yaml.safe_dump(cfg));(run/'scene.json').write_bytes((fixture/'scene.json').read_bytes())
    (run/'world').symlink_to(fixture/'world',target_is_directory=True)
    (run/'policy_revision.txt').write_text('Mixed v1.2; unchanged policy, detector and navigation; simulated camera only\n')
    env=dict(os.environ,BENCH_ROOT=str(run),BENCH_RUN=str(run),BENCH_POLICY='mixed',
        CAMERA_BENCH_WIDTH=str(size),CAMERA_BENCH_HEIGHT=str(size),CAMERA_BENCH_FPS=str(fps),
        CAMERA_BENCH_LOSSLESS='1' if lossless else '0',ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros_logs'))
    print('START',run.name,flush=True)
    try:
        with (run/'observer.log').open('w') as log:
            subprocess.run([sys.executable,'-u',str(HERE/'trial.py')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=500)
        helpers=Path(env['CAMERA_BENCH_HELPERS'])
        with (run/'observer.log').open('a') as log:
            subprocess.run([sys.executable,str(helpers/'2026-11-19_corridor_box/finish_trial.py'),str(run),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
            subprocess.run([sys.executable,str(HERE/'refresh_final.py'),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
    except Exception as exc:
        (run/'trial_error.json').write_text(json.dumps(dict(error=repr(exc),excluded_from_ranking=True))+'\n')
        raise
    finally:cleanup(run)
    print('DONE',run.name,flush=True)
    return run


def main(root):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for handle in locks:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert os.environ['ROS_DOMAIN_ID']=='174' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    clips=[]
    for map_id in (112021,112022):
        clip=native(root,map_id,640,15,'reference',45,True);clips.append(str(clip))
        (root/'reference_clips.json').write_text(json.dumps(clips,indent=2)+'\n')
    subprocess.run([sys.executable,str(HERE/'screen.py'),str(root)],check=True)
    choices=json.loads((root/'screening.json').read_text())['native_candidates']
    runs=[]
    for map_id in (112021,112022):
        for size,fps in [(250,10),*choices]:
            run=native(root,map_id,size,fps,'validation',90)
            runs.append(str(run));(root/'runs.json').write_text(json.dumps(runs,indent=2)+'\n')
    subprocess.run([sys.executable,str(HERE/'summarize.py'),str(root)],check=True)
    (root/'suite_complete.json').write_text(json.dumps(dict(completed=True,native_runs=len(runs),reference_runs=len(clips)))+'\n')


if __name__=='__main__':main(Path(sys.argv[1]).resolve())
