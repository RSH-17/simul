"""Frozen native start screens across 18 conditions, then resume unchanged CEM.

Only training scenes are consumed; holdout worlds remain unused. This owns
dispatch while the previous CEM coordinator is inactive. Existing native jobs
continue, count toward the host limit, and retain their original evidence.
"""
import argparse
import fcntl
import json
from pathlib import Path
import shlex
import subprocess
import time

import yaml

import official_tb4_curriculum_v2 as base
from official_training_input_probe import probe, configuration_gate, motion_gate
from run_floor_pose_regression import probe as floor_probe, floor_gate
from audit_official_crowded_escape import audit as escape_audit
from run_encoder_delta_regression import inspect_slam_transport
from observe_spatial_campaign import native_pid

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-official-constraint-screen-v1.service'
PREVIOUS = 'osp-official-tb4-cem-v2.service'
GATES = ('runtime_configuration_gate', 'velocity_profile_gate', 'scan_adapter_gate',
         'floor_sensor_gate', 'drive_path_gate', 'official_description_gate', 'slam_transport_gate')


def prepare(output, matrix, campaign):
    manifest = json.loads((matrix/'manifest.json').read_text())
    previous = json.loads((campaign/'configuration.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    jobs = []
    for entry in manifest['episodes']:
        if entry['split'] != 'train':
            continue
        scene = yaml.safe_load(Path(entry['file']).read_text())
        if base.digest(Path(entry['file'])) != entry['sha256']:
            raise ValueError('Matrix scene changed')
        number = len(jobs)
        directory = output/'jobs'/f'{number:02d}_{entry["shape"]}_{entry["profile"]}'
        directory.mkdir(parents=True)
        base.save(directory/'policy.json', dict(planner='edge', parameters=previous['baseline_weights'],
                  status='fixed_weights_constraint_screen_not_training'))
        scenario = dict(file=entry['file'], seed=entry['seed'], sha256=entry['sha256'],
            physical_sha256=entry['physical_sha256'], requested_obstacles=entry['obstacles'],
            actual_obstacles=entry['obstacles'], mount_profile='hidden_obstacles', arena=scene['arena'],
            constraint_profile=entry['profile'])
        base.save(directory/'manifest.json', dict(environment='gazebo_harmonic', planner='edge',
            camera_width=320, duration=90., render_backend='software', start_docked=True,
            approach_radius_m=1.5, protocol_version=2, record_video=True, official_robot_unchanged=True,
            sensor_stress_injected=False, policy_sha256=base.digest(directory/'policy.json'),
            scenarios=[scenario], purpose='90-second native start screen only; not full-task training'))
        jobs.append(dict(number=number, role='fixed', phase='diagnostic', generation=0,
            parameters=previous['baseline_weights'], workspace=previous['workspace'], scenario=scenario,
            output=str(directory), status='prepared', stress_profile='official_nominal',
            unit=f'osp-official-constraint-v1-{number:02d}.service'))
    if len(jobs) != 18:
        raise ValueError('Require all 18 shape/profile cells')
    sources = dict(previous['driver_hashes'])
    for name in ('run_official_constraint_screen.py','official_constraint_scenarios.py'):
        path = ROOT/'training_robust'/name
        sources[str(path)] = base.digest(path)
    prepared = [matrix/'manifest.json', *[Path(j['scenario']['file']) for j in jobs],
                *output.glob('jobs/*/manifest.json'), *output.glob('jobs/*/policy.json')]
    base.save(output/'configuration.json', dict(source_hashes=previous['source_hashes'], driver_hashes=sources,
        prepared_hashes={str(p):base.digest(p) for p in prepared}, domains=[228,229], native_limit=2,
        previous_coordinator=PREVIOUS, previous_campaign=str(campaign),
        duration=90., resume_previous_after_completion=True,
        resume_condition='All 18 start screens passed; otherwise preserve CEM for repair', matrix=str(matrix),
        hardware_validated=False, production_promoted=False))
    base.save(output/'state.json', dict(status='prepared', phase='diagnostic', generation=0,
        coordinator_unit=UNIT, jobs=jobs, holdout_consumed=False,
        scope='Native departure screening, not full-task coefficient training'))


def observe(job):
    directory = Path(job['output'])
    live = directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
    if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state') != 'running':
        return
    try:
        if not job.get('runtime_configuration_gate') or not job.get('scan_adapter_gate'):
            report = probe(job['domain'], directory/'runtime_configuration.json')
            check = configuration_gate(report, directory/'policy.json', expected_duration=90.)
            base.save(directory/'runtime_configuration_check.json', check)
            job.update(runtime_configuration_gate=check['passed'], velocity_profile_gate=motion_gate(report),
                       scan_adapter_gate=report.get('scan_adapter',{}).get('passed') is True)
        if not job.get('floor_sensor_gate'):
            job['floor_sensor_gate'] = floor_gate(floor_probe(job['domain'], directory/'floor_probe.json'))
        if not job.get('drive_path_gate'):
            job['drive_path_gate'] = base.drive_path_gate(job)
        if not job.get('official_description_gate'):
            path = directory/'input_inventory.json'
            command = 'source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
                '/usr/bin/python3', str(ROOT/'training_robust/probe_official_tb4.py'),
                '--domain', str(job['domain']), '--output', str(path)])
            subprocess.run(['/bin/bash','-c',command], capture_output=True, check=True, text=True, timeout=25)
            job['official_description_gate'] = json.loads(path.read_text()).get('official_description_exact_match') is True
        if not job.get('slam_transport_gate'):
            pid = native_pid(job['unit'], directory)
            report = inspect_slam_transport(pid) if pid else {}
            base.save(directory/'effective_slam_transport.json', report)
            job['slam_transport_gate'] = bool(report.get('server_environment',{}).get('FASTDDS_BUILTIN_TRANSPORTS')=='UDPv4'
                                              and report.get('shm_threads')==[])
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        job['observation_error'] = repr(exc)


def screen_outcome(audit, input_verified):
    exit_time = audit.get('observed_exit_seconds')
    return bool(input_verified and audit.get('completed') and audit.get('contacts')==0
                and exit_time is not None and 0 <= exit_time <= 90.)


def collect(job):
    base.collect(job)  # Retains independent full-task/physical-credit checks.
    verified = all(job.get(k) is True for k in GATES)
    try:
        audit = escape_audit(Path(job['output']))
    except (OSError, ValueError, KeyError) as exc:
        audit = dict(completed=False, error=repr(exc))
    base.save(Path(job['output'])/'crowded_escape_audit.json', audit)
    job['start_screen_passed'] = screen_outcome(audit, verified)
    job['escape_audit'] = audit
    job['outcome']['start_screen_passed'] = job['start_screen_passed']
    if not verified:
        job['outcome'].update(success=False, infrastructure_failure=True, reason='native_input_unverified')


def verify(config):
    for workspace, hashes in config['source_hashes'].items():
        if base.source_hashes(workspace) != hashes:
            raise RuntimeError('Frozen runtime changed')
    for path, expected in {**config['driver_hashes'], **config['prepared_hashes']}.items():
        if base.digest(Path(path)) != expected:
            raise RuntimeError(f'Frozen experiment source/input changed: {path}')


def run(output):
    config = json.loads((output/'configuration.json').read_text())
    lock = (output/'coordinator.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    state = json.loads((output/'state.json').read_text())
    while any(j['status']!='done' for j in state['jobs']):
        verify(config)
        if base.service_active(PREVIOUS):
            raise RuntimeError('Another coordinator owns dispatch; no new native job started')
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():
                job['status']='running'
            if job['status']=='running':
                if base.service_active(job['unit']):
                    observe(job)
                elif not base.benchmark_output_live(job['output']):
                    collect(job)
        finished = [j for j in state['jobs'] if j['status']=='done']
        if len(finished)>=3 and all(j['outcome']['infrastructure_failure'] for j in finished[-3:]):
            state['status']='needs_infrastructure_repair';base.save(output/'state.json',state);return
        domains = {j.get('domain') for j in state['jobs'] if j['status']=='running'}
        for pid in base.native_processes():
            try:
                argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
                if '--domain' in argv: domains.add(int(argv[argv.index('--domain')+1]))
            except (OSError, ValueError, IndexError):
                continue
        for job in [j for j in state['jobs'] if j['status']=='prepared']:
            if base.occupied_slots(state['jobs'])>=2:
                break
            domain=next((d for d in config['domains'] if d not in domains),None)
            if domain is None: break
            job['domain']=domain;domains.add(domain)
            base.save(output/'state.json',state)
            subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],
                '--output',job['output'],'--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(domain),
                '--wall-timeout','1800','--record-video'],check=True,capture_output=True,text=True)
            job['status']='running';base.save(output/'state.json',state)
        state['status']='running' if any(j['status']=='running' for j in state['jobs']) else 'waiting_for_native_slots'
        base.save(output/'state.json',state)
        time.sleep(5.)
    state['status']='completed'
    state['start_screens_passed']=sum(j.get('start_screen_passed',False) for j in state['jobs'])
    base.save(output/'state.json',state)
    if config['resume_previous_after_completion'] and state['start_screens_passed']==len(state['jobs']):
        # Same unchanged source and original prepared jobs; no result overwrite.
        subprocess.run(['systemctl','--user','start',PREVIOUS],check=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--matrix',type=Path)
    parser.add_argument('--campaign',type=Path)
    parser.add_argument('--run-prepared',action='store_true')
    args=parser.parse_args()
    if args.run_prepared:
        run(args.output.resolve())
    else:
        prepare(args.output.resolve(),args.matrix.resolve(),args.campaign.resolve())
