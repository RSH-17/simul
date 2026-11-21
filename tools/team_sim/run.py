"""One editable Mixed policy in the original track, without the shared VM paths."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from setup import fixtures,home,digest
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('seed',type=int,nargs='?',help='Omit for a fresh random exam. A printed seed reproduces the draw.')
    parser.add_argument('--random-layout',action='store_true',help='Draw a new layout even for legacy seeds 112601–112605')
    parser.add_argument('--markers',choices=('unknown','three'),help='Random exams default to an undisclosed count (1–5)')
    parser.add_argument('--duration',type=float,default=600.)
    parser.add_argument('--domain',type=int,choices=(173,174,175),default=174);parser.add_argument('--check',action='store_true')
    a=parser.parse_args();ws=a.workspace.resolve()
    if not 0<a.duration<=600:parser.error('Duration must be in (0,600]')
    from random_exam import fixture_for,selection
    legacy=json.loads((HERE/'fixtures.json').read_text())['seeds']
    a.seed,randomized,public_count=selection(a.seed,a.random_layout,a.markers,legacy)
    if randomized:
        print(f"Random exam seed {a.seed}, public marker count {public_count or 'undisclosed'}",flush=True)
    assets=fixtures()
    fixture=fixture_for(home(),assets,a.seed,public_count) if randomized else assets/'maps'/f'map-{a.seed}'
    from ament_index_python.packages import get_package_share_directory
    import osp_mission
    package=Path(get_package_share_directory('osp_mission')).resolve()
    if not package.is_relative_to(ws):raise ValueError('Source the selected workspace, not another installed policy')
    for name in ('policy.json','nav2_hardware_profile.yaml'):
        if not (ws/'frozen-config'/name).is_file():raise FileNotFoundError(name)
    for package_name in ('turtlebot4_gz_bringup','irobot_create_gz_bringup','nav2_bringup','slam_toolbox','robot_localization'):
        get_package_share_directory(package_name)
    config=dict(workspace=str(ws),module_file=osp_mission.__file__,seed=a.seed,domain=a.domain,camera=[480,480,5],
        duration_sim_s=a.duration,physical_drive=False,fixture_sha256=digest(fixture/'world/osp_studio.sdf'),
        fixture_mode='randomized_exam' if randomized else 'legacy_five',public_marker_count=public_count)
    if a.check:
        print(json.dumps(dict(config,check='dependencies, fixture integrity, selected runtime resolved; no nodes started'),indent=2));return
    import psutil
    for process in psutil.process_iter():
        try:
            active=process.environ()
            if active.get('ROS_DOMAIN_ID') in ('173','174','175') and active.get('BENCH_RUN'):
                raise SystemExit('Another native benchmark is running on this host; finish it before a team trial.')
        except psutil.Error:pass
    lockdir=home();lockdir.mkdir(parents=True,exist_ok=True)
    with (lockdir/'simulator.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Another team simulation is using this installation.')
        run=home()/'results'/f'team-{ws.name}-track-m{a.seed}-{time.time_ns()}'
        run.mkdir(parents=True)
        for name in ('scene.json','run'):(run/name).symlink_to(fixture/name,target_is_directory=(fixture/name).is_dir())
        import yaml
        scene=yaml.safe_load((fixture/'scene.yaml').read_text());scene['duration']=a.duration
        (run/'scene.yaml').write_text(yaml.safe_dump(scene))
        (run/'world').mkdir()
        tree=ET.parse(fixture/'world/osp_studio.sdf')
        for e in tree.iter():
            if e.text and '@TEAM_MAP_ROOT@' in e.text:e.text='file://'+e.text.replace('@TEAM_MAP_ROOT@',str(fixture))
        tree.write(run/'world/osp_studio.sdf',encoding='utf-8',xml_declaration=True)
        config['effective_world_sha256']=digest(run/'world/osp_studio.sdf')
        config['policy_source_sha256']={str(p.relative_to(ws/'src')):digest(p) for p in (ws/'src/osp_mission').rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json') and '__pycache__' not in p.parts}
        config['external_config_sha256']={name:digest(ws/'frozen-config'/name) for name in ('policy.json','nav2_hardware_profile.yaml')}
        try:config['repository_commit']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True,stderr=subprocess.DEVNULL).strip()
        except subprocess.CalledProcessError:config['repository_commit']=None
        import difflib
        release=REPO/'policy_releases/2026-11-21_mixed_camera_sync_v151'
        patch=[]
        for f in (ws/'src/osp_mission').rglob('*'):
            if not f.is_file() or f.suffix not in ('.py','.yaml','.json') or '__pycache__' in f.parts:continue
            rel=f.relative_to(ws/'src/osp_mission');original=release/'src/osp_mission'/rel
            patch.extend(difflib.unified_diff(original.read_text().splitlines(True) if original.exists() else [],f.read_text().splitlines(True),fromfile='a/'+str(rel),tofile='b/'+str(rel)))
        for name in ('policy.json','nav2_hardware_profile.yaml'):
            patch.extend(difflib.unified_diff((release/name).read_text().splitlines(True),(ws/'frozen-config'/name).read_text().splitlines(True),fromfile='a/frozen-config/'+name,tofile='b/frozen-config/'+name))
        (run/'policy.patch').write_text(''.join(patch))
        (run/'protocol.json').write_text(json.dumps(config,indent=2)+'\n')
        (run/'policy_revision.txt').write_text(ws.name+'\n')
        env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
            RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CAMERA_BENCH_HELPERS=str(HERE/'helpers'),
            CAMERA_BENCH_CARDS=str(home()/'cards'),CAMERA_POLICY_WORKSPACE=str(ws),
            CAMERA_BENCH_WIDTH='480',CAMERA_BENCH_HEIGHT='480',CAMERA_BENCH_FPS='5',
            BENCH_ROOT=str(run),BENCH_RUN=str(run),BENCH_POLICY='mixed',CAMERA_VARIANT='team',CAMERA_PROFILE='balanced',
            ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        env.update(TEAM_RANDOM_EXAM='1' if randomized else '0',OSP_PUBLIC_MARKER_COUNT=str(public_count),
                   SURVEY_EXPERIMENTS=str(REPO/'experiments'))
        for name in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_STATIC_PEERS','ROS_DISCOVERY_SERVER'):env.pop(name,None)
        env['CYCLONEDDS_URI']='file://'+str(REPO/'experiments/2026-11-19_pair_7_103/cyclone.xml')
        (home()/'cards').mkdir(exist_ok=True)
        print('RUN',run,flush=True)
        try:
            with (run/'observer.log').open('w') as log:
                child=subprocess.Popen([sys.executable,'-u',str(HERE/'observer.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:code=child.wait(timeout=2400)
                except (KeyboardInterrupt,subprocess.TimeoutExpired):
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=20)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)
                    raise
                if code:raise RuntimeError('Observer failed; inspect '+str(run/'observer.log'))
                subprocess.run([sys.executable,str(HERE/'helpers/2026-11-19_corridor_box/finish_trial.py'),str(run),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                if randomized:
                    subprocess.run([sys.executable,str(HERE/'random_exam.py'),str(run)],
                                   env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                subprocess.run([sys.executable,str(REPO/'experiments/2026-11-20_v12_camera_rate/refresh_final.py'),str(run)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
            print('COMPLETED',run,flush=True)
        except BaseException as exc:
            (run/'interruption.json').write_text(json.dumps(dict(reason=str(exc),complete=False))+'\n');raise
        finally:
            for process in psutil.process_iter():
                try:
                    if process.environ().get('BENCH_RUN')==str(run):process.terminate()
                except psutil.Error:pass

if __name__=='__main__':main()
