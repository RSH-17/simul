"""Paired map-resolution diagnostic using the unmodified official TB4 robot."""
import argparse
import csv
import fcntl
import json
import math
from pathlib import Path
import shlex
import subprocess
import time

import yaml

from tb4_platform_curriculum import digest, save, source_hashes, occupied_slots, benchmark_output_live
from native_tournament import native_processes, service_active
from official_tb4_probe import probe, configuration_gate, motion_gate
from run_floor_pose_regression import probe as floor_probe, floor_gate
from run_crowded_start_regression_v4 import scene, drive_path_gate

ROOT = Path(__file__).resolve().parents[1]
GATES = ('configuration_gate', 'platform_gate', 'floor_sensor_gate',
         'drive_path_gate', 'official_description_gate')


def prepare(root, candidate, baseline):
    root.mkdir(parents=True, exist_ok=False)
    cfg, geometry = scene(1196100000, .55, math.pi)
    cfg['regression']['scope'] = 'Official dock-compatible crowded-start diagnostic; 90 seconds'
    scene_file = root / 'scene.yaml'
    scene_file.write_text(yaml.safe_dump(cfg, sort_keys=False))
    jobs = []
    for role, workspace, resolution in [('candidate', candidate, .025), ('baseline', baseline, .05)]:
        number = len(jobs)
        directory = root / f'{number:02d}_{role}'
        directory.mkdir()
        (directory / 'policy.json').write_bytes((ROOT / 'policy_releases/2026-11-13_face_intent_v2/policy.json').read_bytes())
        entry = dict(file=str(scene_file), seed=cfg['seed'], sha256=digest(scene_file),
                     arena=cfg['arena'], mount_profile='hidden_obstacles',
                     actual_obstacles=len(cfg['obstacles']), requested_obstacles=len(cfg['obstacles']))
        save(directory / 'manifest.json', dict(environment='gazebo_harmonic', planner='edge',
            camera_width=320, duration=90., render_backend='software', start_docked=True,
            approach_radius_m=1.5, protocol_version=2, policy_sha256=digest(directory / 'policy.json'),
            scenarios=[entry], purpose='Official paired resolution diagnostic; not full-task training'))
        jobs.append(dict(number=number, role=role, phase='diagnostic', generation=0,
            workspace=str(workspace), output=str(directory), scenario=entry, geometry=geometry,
            status='prepared', map_resolution=resolution, unit=f'osp-official-crowded-v1-{number:02d}.service'))
    helpers = ['tb4_platform_curriculum.py', 'tb4_platform_probe.py', 'official_tb4_probe.py',
        'probe_official_tb4.py', 'runtime_configuration_v2.py', 'native_tournament.py',
        'run_floor_pose_regression.py', 'run_native_service.py', 'gazebo_benchmark.py',
        'run_crowded_start_regression_v4.py', 'capture_native_failures.py',
        'observe_spatial_campaign.py', 'probe_sim_drive_path.py']
    save(root / 'configuration.json', dict(source_hashes={str(w): source_hashes(w) for w in [candidate, baseline]},
        driver_sha256=digest(Path(__file__)), support_hashes={str(ROOT/'training_robust'/n):digest(ROOT/'training_robust'/n) for n in helpers},
        prepared_hashes={str(p):digest(p) for p in [scene_file, *root.glob('*/manifest.json'), *root.glob('*/policy.json')]},
        domains=[228, 229], native_limit=2, duration=90., hardware_validated=False))
    save(root / 'state.json', dict(status='prepared', phase='diagnostic', generation=0, jobs=jobs,
        coordinator_unit='osp-official-crowded-comparison-v1.service',
        scope='Paired map-resolution diagnostic; policy weight training has not resumed'))


def observe(job):
    directory = Path(job['output'])
    if not (directory / 'episodes' / str(job['scenario']['seed']) / 'live_status.json').exists():
        return
    try:
        if not job.get('configuration_gate') or not job.get('platform_gate'):
            report = probe(job['domain'], directory / 'runtime_configuration.json')
            checked = configuration_gate(report, directory / 'policy.json', expected_duration=90.,
                                         expected_resolution=job['map_resolution'])
            save(directory / 'runtime_configuration_check.json', checked)
            job.update(configuration_gate=checked['passed'], platform_gate=motion_gate(report))
        if not job.get('floor_sensor_gate'):
            job['floor_sensor_gate'] = floor_gate(floor_probe(job['domain'], directory / 'floor_probe.json'))
        if not job.get('drive_path_gate'):
            job['drive_path_gate'] = drive_path_gate(job)
        if not job.get('official_description_gate'):
            output = directory / 'input_inventory.json'
            command = 'source /opt/ros/jazzy/setup.bash\nexec ' + shlex.join([
                '/usr/bin/python3', str(ROOT / 'training_robust/probe_official_tb4.py'),
                '--domain', str(job['domain']), '--output', str(output)])
            subprocess.run(['/bin/bash', '-c', command], check=True, capture_output=True, text=True, timeout=25)
            job['official_description_gate'] = json.loads(output.read_text()).get('official_description_exact_match') is True
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        job['observation_error'] = repr(exc)


def collect(job):
    directory = Path(job['output'])
    episode = directory / 'episodes' / str(job['scenario']['seed'])
    path = episode / 'evaluation.json'
    result = json.loads(path.read_text()) if path.exists() else {}
    trace = episode / 'ground_truth_trajectory.csv'
    rows = list(csv.DictReader(trace.open())) if trace.exists() else []
    first = float(rows[0]['sim_seconds']) if rows else 0.
    exit_time = next((float(r['sim_seconds']) - first for r in rows if float(r['world_x']) >= 8.3), None)
    distance = sum(math.hypot(float(b['world_x'])-float(a['world_x']), float(b['world_y'])-float(a['world_y']))
                   for a, b in zip(rows, rows[1:]))
    valid = all(job.get(k) is True for k in GATES)
    completed = result.get('reason') in ('timeout', 'all_targets_found')
    job.update(status='done', outcome=dict(escaped=bool(valid and completed and exit_time is not None
        and 0 <= exit_time <= 90. and result.get('collisions') == 0), valid_input=valid,
        exit_seconds=exit_time, actual_path_m=distance, contacts=result.get('collisions'),
        reason=result.get('reason'), full_task_success=result.get('success', False),
        qualification_scope='90-second crowded-start diagnostic only',
        exit_timing_origin='first independent ground-truth sample, including startup and undock'))
    save(directory / 'crowded_start_audit.json', job['outcome'])


def run(root):
    lock = (root / 'coordinator.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    config = json.loads((root / 'configuration.json').read_text())
    state = json.loads((root / 'state.json').read_text())
    while any(j['status'] != 'done' for j in state['jobs']):
        for workspace, expected in config['source_hashes'].items():
            if source_hashes(workspace) != expected:
                raise RuntimeError('Frozen runtime changed')
        if digest(Path(__file__)) != config['driver_sha256']:
            raise RuntimeError('Frozen driver changed')
        for path, expected in {**config['support_hashes'], **config['prepared_hashes']}.items():
            if digest(Path(path)) != expected:
                raise RuntimeError(f'Frozen experiment input changed: {path}')
        for job in state['jobs']:
            if job['status'] == 'prepared' and (Path(job['output']) / 'service.json').exists():
                job['status'] = 'running'
            if job['status'] == 'running':
                if service_active(job['unit']):
                    observe(job)
                elif not benchmark_output_live(job['output']):
                    collect(job)
        occupied = occupied_slots(state['jobs'])
        domains = {j['domain'] for j in state['jobs'] if j['status'] == 'running'}
        for job in [j for j in state['jobs'] if j['status'] == 'prepared'][:max(0, 2-occupied)]:
            if len(native_processes()) >= 2:
                break
            job['domain'] = next(d for d in config['domains'] if d not in domains)
            domains.add(job['domain'])
            save(root / 'state.json', state)
            subprocess.run(['/usr/bin/python3', str(ROOT/'training_robust/run_native_service.py'),
                '--unit', job['unit'].removesuffix('.service'), '--workspace', job['workspace'],
                '--output', job['output'], '--', '/usr/bin/python3', str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output', job['output'], '--run-prepared', '--domain', str(job['domain']),
                '--wall-timeout', '1500', '--record-video'], check=True, capture_output=True, text=True)
            job['status'] = 'running'
            save(root / 'state.json', state)
        state['status'] = 'running' if any(j['status'] == 'running' for j in state['jobs']) else 'waiting_for_native_slots'
        save(root / 'state.json', state)
        time.sleep(5)
    state['status'] = 'completed'
    save(root / 'state.json', state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--run-prepared', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if not args.run_prepared:
        prepare(args.output.resolve(), args.candidate.resolve(), args.baseline.resolve())
    if not args.prepare_only:
        run(args.output.resolve())
