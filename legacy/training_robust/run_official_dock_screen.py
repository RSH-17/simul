"""Two frozen dock-repair departure trials after existing native slots drain."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time

import run_official_constraint_screen as screen
import run_official_frontier_comparison as full
import official_local_grid_probe as grid
import official_dock_map_probe as dock
from audit_pose_trace import audit as audit_pose

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-official-dock-screen-v1.service'
FRONTIER='osp-official-frontier-comparison-v1.service'
BLOCKERS=[FRONTIER,*full.BLOCKERS]
GATES=(*screen.GATES,'navigation_grid_gate','dock_map_gate','candidate_output_profiles_gate')
DELTA={'osp_mission/mission.py','osp_mission/dock_geometry.py','config/tb4_nav2.yaml',
       'launch/robot.launch.py','test/test_start_dock_status.py','test/test_dock_anchor.py'}


def prepare(output,workspace,matrix,frontier):
    previous=json.loads((matrix/'configuration.json').read_text());state=json.loads((matrix/'state.json').read_text())
    screen.verify(previous)
    parent=Path(state['jobs'][4]['workspace'])
    old=screen.base.source_hashes(parent);new=screen.base.source_hashes(workspace)
    if {k for k in old.keys()|new.keys() if old.get(k)!=new.get(k)}!=DELTA:
        raise ValueError('Candidate must isolate the dock repair from the failed parent')
    if not (workspace/'install/setup.bash').exists():raise ValueError('Build candidate first')
    originals=[state['jobs'][i] for i in [4,0]]
    for j in originals:
        if j['status']!='done' or j['outcome']['infrastructure_failure']:
            raise ValueError('Require completed, verified original baselines')
    output.mkdir(parents=True,exist_ok=False);jobs=[]
    for original in originals:
        number=len(jobs);directory=output/'jobs'/f'{number:02d}_{original["scenario"]["constraint_profile"]}'
        directory.mkdir(parents=True);prior=Path(original['output'])
        if json.loads((prior/'manifest.json').read_text()).get('duration')!=90.:
            raise ValueError('This comparison preserves 90-second departure baselines')
        for name in ['policy.json','manifest.json']:(directory/name).write_bytes((prior/name).read_bytes())
        jobs.append(dict(number=number,role='candidate',phase='diagnostic',generation=0,workspace=str(workspace),
            output=str(directory),scenario=original['scenario'],status='prepared',stress_profile='official_nominal',
            unit=f'osp-official-dock-v1-{number:02d}.service',original_outcome=original['outcome'],original_output=str(prior)))
    drivers=dict(previous['driver_hashes'])
    for name in ['run_official_dock_screen.py','official_dock_map_probe.py','run_official_frontier_comparison.py','audit_pose_trace.py']:
        p=ROOT/'training_robust'/name;drivers[str(p)]=screen.base.digest(p)
    files=[*output.glob('jobs/*/policy.json'),*output.glob('jobs/*/manifest.json'),
           *[Path(j['scenario']['file']) for j in jobs]]
    screen.base.save(output/'configuration.json',dict(source_hashes={str(parent):old,str(workspace):new},driver_hashes=drivers,
        prepared_hashes={str(p):screen.base.digest(p) for p in files},domains=[228,229],native_limit=2,duration=90.,
        frontier_campaign=str(frontier),resume_coordinator=FRONTIER,holdout_consumed=False,weights_retrained=False,
        production_promoted=False,hardware_validated=False))
    screen.base.save(output/'state.json',dict(status='prepared',phase='diagnostic',generation=0,
        coordinator_unit=UNIT,jobs=jobs,scope='Same-parent dock repair: sideways failure and narrow-start regression; 90s only'))


def observe(job):
    screen.probe=grid.probe;screen.configuration_gate=grid.configuration_gate
    screen.observe(job)
    directory=Path(job['output']);p=directory/'runtime_configuration.json'
    if p.exists():
        result=grid.navigation_grid_gate(json.loads(p.read_text()))
        job['navigation_grid_gate']=result['passed'];screen.base.save(directory/'navigation_grid_check.json',result)
    live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
    if (not job.get('dock_map_gate') and live.exists()
            and json.loads(live.read_text()).get('status',{}).get('state')=='running'):
        try:
            report=dock.probe(job['domain'],directory/'dock_map_probe.json')
            job['dock_map_gate']=dock.gate(report)['passed']
        except (OSError,ValueError,subprocess.SubprocessError) as exc:job['dock_observation_error']=repr(exc)


def collect(job):
    screen.collect(job)
    job['candidate_output_profiles_gate']=screen.base.candidate_profiles(job['output'])
    verified=all(job.get(k) is True for k in GATES)
    job['start_screen_passed']=screen.screen_outcome(job['escape_audit'],verified)
    job['outcome']['start_screen_passed']=job['start_screen_passed']
    if not verified:job['outcome'].update(success=False,infrastructure_failure=True,reason='dock_or_native_inputs_unverified')
    p=Path(job['output']);invalidation=p/'environment_invalidation.json'
    if invalidation.exists() and json.loads(invalidation.read_text()).get('policy_training_sample_valid') is False:
        job['outcome'].update(success=False,infrastructure_failure=True,reason='environment_invalidation',start_screen_passed=False)
        job['start_screen_passed']=False
    try:
        report=audit_pose(p/'episodes'/str(job['scenario']['seed']));screen.base.save(p/'pose_audit.json',report)
        job['localization_audit']=report['pose_errors']
    except (OSError,ValueError,KeyError,IndexError) as exc:job['diagnostic_audit_error']=repr(exc)


def collect_drained_frontier(path):
    # The original coordinator is held, but its existing native processes run
    # unchanged. Collect their original rubric as they finish; never cancel them.
    lock=(path/'coordinator.lock').open('a')
    with lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        state=json.loads((path/'state.json').read_text())
        for job in state['jobs']:
            if job['status']=='running' and not screen.base.service_active(job['unit']) and not screen.base.benchmark_output_live(job['output']):
                full.collect(job)
        state['status']='held_for_dock_screen';screen.base.save(path/'state.json',state)


def run(output):
    lock=(output/'coordinator.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    config=json.loads((output/'configuration.json').read_text());state=json.loads((output/'state.json').read_text())
    while any(j['status']!='done' for j in state['jobs']):
        screen.verify(config)
        if any(screen.base.service_active(unit) for unit in BLOCKERS):raise RuntimeError('Another dispatcher is active')
        collect_drained_frontier(Path(config['frontier_campaign']))
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():job['status']='running'
            if job['status']=='running':
                if screen.base.service_active(job['unit']):observe(job)
                elif not screen.base.benchmark_output_live(job['output']):collect(job)
        for job in [j for j in state['jobs'] if j['status']=='prepared']:
            if screen.base.occupied_slots(state['jobs'])>=2:break
            domains={j.get('domain') for j in state['jobs'] if j['status']=='running'}
            for pid in screen.base.native_processes():
                try:
                    argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
                    if '--domain' in argv:domains.add(int(argv[argv.index('--domain')+1]))
                except (OSError,ValueError,IndexError):continue
            domain=next((d for d in config['domains'] if d not in domains),None)
            if domain is None:break
            job['domain']=domain;screen.base.save(output/'state.json',state)
            subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],'--output',job['output'],
                '--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),'--output',job['output'],
                '--run-prepared','--domain',str(domain),'--wall-timeout','1800','--record-video'],
                check=True,capture_output=True,text=True)
            job['status']='running';screen.base.save(output/'state.json',state)
        state['status']='running' if any(j['status']=='running' for j in state['jobs']) else 'waiting_for_native_slots'
        screen.base.save(output/'state.json',state);time.sleep(5.)
    state['status']='completed';screen.base.save(output/'state.json',state)
    subprocess.run(['systemctl','--user','start',config['resume_coordinator']],check=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    for n in ['workspace','matrix','frontier']:p.add_argument('--'+n,type=Path)
    p.add_argument('--run-prepared',action='store_true');a=p.parse_args()
    if a.run_prepared:run(a.output.resolve())
    else:prepare(a.output.resolve(),a.workspace.resolve(),a.matrix.resolve(),a.frontier.resolve())
