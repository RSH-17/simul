"""Prepared full-task comparison of integrated TB4 policy and stricter loop gates.

Launch requires the independent v16 departure prerequisite and sole ownership
of dispatch. This does not resume older policy campaigns or promote a winner.
"""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time

import yaml

import run_official_frontier_comparison as full
import official_integrated_probe as probe
import official_dock_map_probe as dock

screen = full.screen
ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-official-integrated-comparison-v1.service'
BLOCKERS = list(dict.fromkeys([*full.BLOCKERS, full.UNIT,
    'osp-official-pocket-screen-v1.service', 'osp-official-frontier-evidence-screen-v1.service']))
GATES = (*full.GATES, 'dock_map_gate', 'loop_settings_gate', 'deadline_settings_gate')


def prerequisite_report(directory):
    state = json.loads((directory/'state.json').read_text())
    jobs = state['jobs']
    passed = (state['status'] == 'completed' and len(jobs) == 2
              and all(j['status'] == 'done' and j.get('start_screen_passed') is True
                      and not j['outcome']['infrastructure_failure'] for j in jobs))
    return dict(passed=passed, status=state['status'],
        rows=[dict(number=j['number'], status=j['status'],
                   departure_passed=j.get('start_screen_passed')) for j in jobs],
        scope='Departure and zero-contact prerequisite only; not full mission qualification')


def prepare(output, baseline, candidate, original, prerequisite, policy):
    old, new = screen.base.source_hashes(baseline), screen.base.source_hashes(candidate)
    if {k for k in old.keys() | new.keys() if old.get(k) != new.get(k)} != {'config/tb4_slam.yaml'}:
        raise ValueError('Require isolated loop settings between integrated sources')
    configs = [yaml.safe_load((w/'src/osp_mission/config/tb4_slam.yaml').read_text())
               for w in [baseline, candidate]]
    for config, expected in zip(configs, [(.35, .45, 3.), (.55, .65, 1.)]):
        params = config['slam_toolbox']['ros__parameters']
        if params.get('do_loop_closing') is not True:
            raise ValueError('Loop closure must remain enabled')
        observed = tuple(params.pop(key) for key in probe.LOOP_KEYS[1:])
        if observed != expected:
            raise ValueError('Require the predeclared loop-gate comparison')
    if configs[0] != configs[1]:
        raise ValueError('Other SLAM settings must remain identical')
    for workspace in [baseline, candidate]:
        if not (workspace/'install/setup.bash').exists():
            raise ValueError('Build both integrated candidates first')
    policy_data = json.loads(policy.read_text())
    if policy_data.get('settings') != {'deadline_aware': True} or len(policy_data['parameters']) != 5:
        raise ValueError('Require the integrated fixed-weight deadline policy')
    prior = json.loads((original/'configuration.json').read_text())
    screen.verify(prior)
    state = json.loads((original/'state.json').read_text())
    output.mkdir(parents=True, exist_ok=False)
    jobs = []
    for number in [2, 0, 4]:  # Exercise the diagnosed U failure first.
        previous = next(j for j in state['jobs'] if j['number'] == number)
        scene = previous['scenario']
        if screen.base.digest(Path(scene['file'])) != scene['sha256']:
            raise ValueError('Original development scene changed')
        original_manifest = json.loads((Path(previous['output'])/'manifest.json').read_text())
        if original_manifest['duration'] != 600.:
            raise ValueError('Keep the full task duration')
        for role, workspace in [('baseline', baseline), ('candidate', candidate)]:
            index = len(jobs)
            directory = output/'jobs'/f'{index:02d}_{scene["arena"]["shape"]}_{role}'
            directory.mkdir(parents=True)
            (directory/'policy.json').write_bytes(policy.read_bytes())
            screen.base.save(directory/'manifest.json', dict(original_manifest,
                policy_sha256=screen.base.digest(directory/'policy.json'),
                purpose='Integrated fixed-weight 600-second task; loop-gate comparison; development only'))
            slam = yaml.safe_load((workspace/'src/osp_mission/config/tb4_slam.yaml').read_text())['slam_toolbox']['ros__parameters']
            jobs.append(dict(number=index, role=role, phase='development', generation=0,
                workspace=str(workspace), output=str(directory), scenario=scene, status='prepared',
                stress_profile='official_nominal', unit=f'osp-official-integrated-v1-{index:02d}.service',
                expected_loop={key: slam[key] for key in probe.LOOP_KEYS}))
    drivers = dict(prior['driver_hashes'])
    for name in ['run_official_integrated_comparison.py', 'official_integrated_probe.py',
                 'official_dock_map_probe.py', 'run_official_frontier_comparison.py']:
        path = ROOT/'training_robust'/name
        drivers[str(path)] = screen.base.digest(path)
    prepared = [policy, *output.glob('jobs/*/policy.json'), *output.glob('jobs/*/manifest.json'),
                *[Path(j['scenario']['file']) for j in jobs]]
    screen.base.save(output/'configuration.json', dict(source_hashes={str(baseline): old, str(candidate): new},
        driver_hashes=drivers, prepared_hashes={str(p): screen.base.digest(p) for p in prepared},
        domains=[230, 231], native_limit=2, duration=600., wall_timeout_s=7200,
        prerequisite=str(prerequisite), prerequisite_source_sha256=screen.base.digest(prerequisite/'configuration.json'),
        weights_retrained=False, holdout_consumed=False, production_promoted=False, hardware_validated=False,
        native_qualification_pending=True, resume_older_campaigns=False))
    screen.base.save(output/'state.json', dict(status='prepared_pending_departure_prerequisite',
        phase='development', generation=0, coordinator_unit=UNIT, jobs=jobs,
        scope='Three shapes, integrated source pair, full original success rubric'))


def observe(job):
    screen.probe = probe.probe
    screen.configuration_gate = full.full_configuration_gate
    screen.observe(job)
    directory = Path(job['output'])
    path = directory/'runtime_configuration.json'
    if path.exists():
        report = json.loads(path.read_text())
        job['navigation_grid_gate'] = full.grid.navigation_grid_gate(report)['passed']
        result = probe.loop_gate(report, job['expected_loop'])
        job['loop_settings_gate'] = result['passed']
        screen.base.save(directory/'loop_settings_check.json', result)
        if not result['passed']:
            job['runtime_configuration_gate'] = False  # Re-probe missing/incorrect settings.
    live = directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
    if live.exists():
        status = json.loads(live.read_text()).get('status', {})
        job['deadline_settings_gate'] = status.get('planning_settings') == {'deadline_aware': True}
        if not job.get('dock_map_gate') and status.get('state') == 'running':
            try:
                job['dock_map_gate'] = dock.gate(dock.probe(job['domain'], directory/'dock_map_probe.json'))['passed']
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                job['dock_observation_error'] = repr(exc)


def collect(job):
    full.collect(job)
    if not all(job.get(k) is True for k in GATES):
        job['raw_outcome_before_integrated_gate'] = dict(job['outcome'])
        job['outcome'].update(success=False, infrastructure_failure=True, reason='integrated_runtime_gate_unverified')


def run(output):
    with (output/'coordinator.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        config = json.loads((output/'configuration.json').read_text())
        state = json.loads((output/'state.json').read_text())
        prerequisite = Path(config['prerequisite'])
        if screen.base.digest(prerequisite/'configuration.json') != config['prerequisite_source_sha256']:
            raise ValueError('Departure prerequisite changed')
        screen.verify(json.loads((prerequisite/'configuration.json').read_text()))
        report = prerequisite_report(prerequisite)
        screen.base.save(output/'prerequisite_check.json', report)
        if not report['passed']:
            state['status'] = 'waiting_for_verified_departure'
            screen.base.save(output/'state.json', state)
            return
        while any(j['status'] != 'done' for j in state['jobs']):
            screen.verify(config)
            if any(screen.base.service_active(unit) for unit in BLOCKERS):
                raise RuntimeError('Another dispatcher is active; handoff must preserve original native jobs')
            for job in state['jobs']:
                if job['status'] == 'prepared' and (Path(job['output'])/'service.json').exists():
                    job['status'] = 'running'
                if job['status'] == 'running':
                    if screen.base.service_active(job['unit']):
                        observe(job)
                    elif not screen.base.benchmark_output_live(job['output']):
                        collect(job)
            done = [j for j in state['jobs'] if j['status'] == 'done']
            screen.base.save(output/'results.json', dict(complete=len(done) == len(state['jobs']),
                rows=[dict(number=j['number'], role=j['role'], shape=j['scenario']['arena']['shape'],
                           outcome=j['outcome']) for j in done], production_promoted=False))
            if len(done) >= 3 and all(j['outcome']['infrastructure_failure'] for j in done[-3:]):
                state['status'] = 'needs_infrastructure_repair'
                screen.base.save(output/'state.json', state)
                return
            for job in [j for j in state['jobs'] if j['status'] == 'prepared']:
                if screen.base.occupied_slots(state['jobs']) >= 2:
                    break
                # Existing external native jobs own both their slot and domain.
                domains = {j.get('domain') for j in state['jobs'] if j['status'] == 'running'}
                for pid in screen.base.native_processes():
                    try:
                        argv = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
                        if '--domain' in argv:
                            domains.add(int(argv[argv.index('--domain')+1]))
                    except (OSError, ValueError, IndexError):
                        continue
                domain = next((d for d in config['domains'] if d not in domains), None)
                if domain is None:
                    break
                job['domain'] = domain
                screen.base.save(output/'state.json', state)
                subprocess.run(['/usr/bin/python3', str(ROOT/'training_robust/run_native_service.py'),
                    '--unit', job['unit'].removesuffix('.service'), '--workspace', job['workspace'],
                    '--output', job['output'], '--', '/usr/bin/python3', str(ROOT/'training_robust/gazebo_benchmark.py'),
                    '--output', job['output'], '--run-prepared', '--domain', str(domain),
                    '--wall-timeout', '7200', '--record-video'], check=True, capture_output=True, text=True)
                job['status'] = 'running'
                screen.base.save(output/'state.json', state)
            state['status'] = 'running' if any(j['status'] == 'running' for j in state['jobs']) else 'waiting_for_native_slots'
            screen.base.save(output/'state.json', state)
            time.sleep(5.)
        state['status'] = 'completed'
        screen.base.save(output/'state.json', state)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    for name in ['baseline', 'candidate', 'original', 'prerequisite', 'policy']:
        p.add_argument('--'+name, type=Path)
    p.add_argument('--run-prepared', action='store_true')
    a = p.parse_args()
    if a.run_prepared:
        run(a.output.resolve())
    else:
        prepare(*[getattr(a, k).resolve() for k in ['output', 'baseline', 'candidate', 'original', 'prerequisite', 'policy']])
