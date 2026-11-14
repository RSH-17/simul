"""Six full 600-second paired native trials, with frozen code and shared scenes."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time

from launch_official_tb4_curriculum_v2 import qualify
import run_official_constraint_screen as screen
import official_local_grid_probe as grid
from audit_pose_trace import audit as audit_pose

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-official-frontier-comparison-v1.service'
MATRIX='osp-official-local-grid-matrix-v1.service'
BLOCKERS=[MATRIX,'osp-official-camera-view-screen-v1.service',
          'osp-official-tb4-cem-v1.service','osp-official-tb4-cem-v2.service']
GATES=(*screen.GATES,'navigation_grid_gate','candidate_output_profiles_gate')
EXPECTED_DELTA={'osp_mission/edge_policy.py','osp_mission/frontier_support.py','test/test_frontier_support.py'}


def full_configuration_gate(report,policy,**kwargs):
    return grid.configuration_gate(report,policy,expected_duration=600.)


def compare_sources(baseline,candidate):
    old=screen.base.source_hashes(baseline);new=screen.base.source_hashes(candidate)
    changed={p for p in old.keys()|new.keys() if old.get(p)!=new.get(p)}
    if changed!=EXPECTED_DELTA:raise ValueError('Pair differs beyond the intended frontier feature and its tests')
    return {str(baseline):old,str(candidate):new}


def prepare(output,baseline,candidate,comparison,matrix):
    qualification=qualify(baseline,comparison)
    sources=compare_sources(baseline,candidate)
    if not (candidate/'install/setup.bash').exists():raise ValueError('Candidate must be built first')
    prior=json.loads((matrix/'configuration.json').read_text())
    matrix_state=json.loads((matrix/'state.json').read_text())
    output.mkdir(parents=True,exist_ok=False);jobs=[]
    for scene_number in (0,9,17):
        original=matrix_state['jobs'][scene_number];scene=original['scenario']
        original_manifest=json.loads((Path(original['output'])/'manifest.json').read_text())
        if screen.base.digest(Path(scene['file']))!=scene['sha256']:raise ValueError('Scenario changed')
        for role,workspace in [('baseline',baseline),('candidate',candidate)]:
            number=len(jobs);directory=output/'jobs'/f'{number:02d}_{scene["arena"]["shape"]}_{role}'
            directory.mkdir(parents=True)
            (directory/'policy.json').write_bytes((comparison/'policy.json').read_bytes())
            manifest=dict(original_manifest,duration=600.,policy_sha256=screen.base.digest(directory/'policy.json'),
                purpose='Full 600-second paired algorithm development; same weights; not held-out validation')
            screen.base.save(directory/'manifest.json',manifest)
            jobs.append(dict(number=number,role=role,phase='development',generation=0,
                workspace=str(workspace),output=str(directory),scenario=scene,status='prepared',
                stress_profile='official_nominal',unit=f'osp-official-frontier-v1-{number:02d}.service'))
    drivers=dict(prior['driver_hashes'])
    for name in ['run_official_frontier_comparison.py','official_local_grid_probe.py','audit_pose_trace.py']:
        path=ROOT/'training_robust'/name;drivers[str(path)]=screen.base.digest(path)
    prepared=[*output.glob('jobs/*/policy.json'),*output.glob('jobs/*/manifest.json'),
              *[Path(j['scenario']['file']) for j in jobs]]
    screen.base.save(output/'configuration.json',dict(source_hashes=sources,driver_hashes=drivers,
        prepared_hashes={str(p):screen.base.digest(p) for p in prepared},domains=[230,231],native_limit=2,
        duration=600.,wall_timeout_s=7200,startup_qualification=qualification,
        candidate_native_qualified_before_trial=False,resume_matrix_after_completion=MATRIX,
        holdout_consumed=False,weights_retrained=False,production_promoted=False,hardware_validated=False,
        primary_comparison='Supported unknown-region frontier reward; all other runtime source and weights equal'))
    screen.base.save(output/'state.json',dict(status='prepared',phase='development',generation=0,
        coordinator_unit=UNIT,jobs=jobs,scope='Three shapes, paired fixed weights, complete hidden-marker task'))


def observe(job):
    screen.probe=grid.probe;screen.configuration_gate=full_configuration_gate
    screen.observe(job)
    path=Path(job['output'])/'runtime_configuration.json'
    if path.exists():
        report=grid.navigation_grid_gate(json.loads(path.read_text()))
        job['navigation_grid_gate']=report['passed'];screen.base.save(path.parent/'navigation_grid_check.json',report)


def collect(job):
    screen.base.collect(job)
    job['candidate_output_profiles_gate']=screen.base.candidate_profiles(job['output'])
    if not all(job.get(k) is True for k in GATES):
        job['raw_outcome_before_input_gate']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='native_inputs_or_output_profile_unverified')
    invalidation=Path(job['output'])/'environment_invalidation.json'
    if invalidation.exists() and json.loads(invalidation.read_text()).get('policy_training_sample_valid') is False:
        job['raw_outcome_before_environment_invalidation']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='environment_invalidation')
    episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    try:
        report=audit_pose(episode);screen.base.save(Path(job['output'])/'pose_audit.json',report)
        job['localization_audit']=report['pose_errors']
        departure=screen.escape_audit(Path(job['output']))
        screen.base.save(Path(job['output'])/'crowded_escape_audit.json',departure)
        job['escape_audit']=departure
    except (OSError,ValueError,KeyError,IndexError) as exc:job['diagnostic_audit_error']=repr(exc)


def run(output):
    lock=(output/'coordinator.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    config=json.loads((output/'configuration.json').read_text())
    state=json.loads((output/'state.json').read_text())
    while any(j['status']!='done' for j in state['jobs']):
        screen.verify(config)
        if any(screen.base.service_active(unit) for unit in BLOCKERS):
            raise RuntimeError('Another dispatcher is active; preserve the two-native host limit')
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():job['status']='running'
            if job['status']=='running':
                if screen.base.service_active(job['unit']):observe(job)
                elif not screen.base.benchmark_output_live(job['output']):collect(job)
        done=[j for j in state['jobs'] if j['status']=='done']
        screen.base.save(output/'results.json',dict(complete=len(done)==6,
            rows=[dict(number=j['number'],role=j['role'],shape=j['scenario']['arena']['shape'],
                       outcome=j['outcome']) for j in done],production_promoted=False))
        if len(done)>=3 and all(j['outcome']['infrastructure_failure'] for j in done[-3:]):
            state['status']='needs_infrastructure_repair';screen.base.save(output/'state.json',state);return
        domains={j.get('domain') for j in state['jobs'] if j['status']=='running'}
        for pid in screen.base.native_processes():
            try:
                argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
                if '--domain' in argv:domains.add(int(argv[argv.index('--domain')+1]))
            except (OSError,ValueError,IndexError):continue
        for job in [j for j in state['jobs'] if j['status']=='prepared']:
            if screen.base.occupied_slots(state['jobs'])>=2:break
            domain=next((d for d in config['domains'] if d not in domains),None)
            if domain is None:break
            job['domain']=domain;domains.add(domain);screen.base.save(output/'state.json',state)
            subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],
                '--output',job['output'],'--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(domain),
                '--wall-timeout','7200','--record-video'],check=True,capture_output=True,text=True)
            job['status']='running';screen.base.save(output/'state.json',state)
        state['status']='running' if any(j['status']=='running' for j in state['jobs']) else 'waiting_for_native_slots'
        screen.base.save(output/'state.json',state);time.sleep(5.)
    state['status']='completed';screen.base.save(output/'state.json',state)
    subprocess.run(['systemctl','--user','start',config['resume_matrix_after_completion']],check=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    for name in ['baseline','candidate','comparison','matrix']:p.add_argument('--'+name,type=Path)
    p.add_argument('--run-prepared',action='store_true');a=p.parse_args()
    if a.run_prepared:run(a.output.resolve())
    else:prepare(a.output.resolve(),a.baseline.resolve(),a.candidate.resolve(),a.comparison.resolve(),a.matrix.resolve())
