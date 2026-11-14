"""Frozen, sequential loop-gate component comparisons on completed official runs.

The mapper receives only recorded ranges, odometry and its saved parameters.
This runner has no evaluator imports and cannot claim navigation success.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2)+'\n')
    temp.replace(path)


def verify(hashes):
    for name, expected in hashes.items():
        if digest(Path(name)) != expected:
            raise ValueError('Frozen component input changed: '+name)


def prepare(campaign, executable, solver_prefix, original_u_replay, output):
    prior = json.loads((original_u_replay/'configuration.json').read_text())
    verify(prior['source_sha256'])
    if not (json.loads((original_u_replay/'baseline.result.json').read_text())['status'] == 'completed'
            and prior['binary'] == str(executable) and prior['solver_prefix'] == str(solver_prefix)):
        raise ValueError('Require the completed baseline with the same mapper and solver')
    state = json.loads((campaign/'state.json').read_text())
    originals = state['jobs'][:4]
    if len(originals) != 4 or any(j['status'] != 'done' or j['outcome']['infrastructure_failure'] for j in originals):
        raise ValueError('Require four completed, verified development recordings')
    if {j['scenario']['arena']['shape'] for j in originals} != {'c_shape', 'u_shape'}:
        raise ValueError('Require the completed C/U pairs')
    output.mkdir(parents=True, exist_ok=False)
    driver = Path(__file__).resolve()
    files = [driver, executable, solver_prefix/'lib/libosp_solver_budget.so',
             driver.with_name('replay_karto_component.cpp')]
    jobs = []
    for job in originals:
        episode = Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        datasets = list(episode.glob('*/map.data'))
        if len(datasets) != 1:
            raise ValueError('Require one finalized dataset')
        data = datasets[0]
        posegraph = data.with_suffix('.posegraph')
        directory = output/f'case_{job["number"]:02d}'
        directory.mkdir()
        files += [data, posegraph]
        config = dict(variants=['baseline', 'strict_loop'], data=str(data), posegraph=str(posegraph),
            binary=str(executable), solver_prefix=str(solver_prefix), driver=str(driver),
            original_mapper_parameters_copied=True, solver_threads=2, ground_truth_input=False)
        if job['number'] == 2:
            if prior['data'] != str(data) or prior['posegraph'] != str(posegraph):
                raise ValueError('Reused U baseline has different inputs')
            copied = {}
            for suffix in ['online.csv', 'events.log', 'parameters.txt', 'result.json', 'log']:
                source = original_u_replay/('baseline.'+suffix)
                dest = directory/source.name
                shutil.copyfile(source, dest)
                copied[str(source)] = digest(source)
                files += [source, dest]
            config['baseline_reused_from'] = dict(directory=str(original_u_replay), source_sha256=copied)
        save(directory/'configuration.json', config)
        files.append(directory/'configuration.json')
        jobs.append(dict(number=job['number'], shape=job['scenario']['arena']['shape'],
            episode=str(episode), scenario_file=job['scenario']['file'], output=str(directory), status='prepared'))
    # Input file hashes are checked before every arm; the study does not edit
    # native sources, the saved graph, any policy or the original evaluation.
    save(output/'configuration.json', dict(source_sha256={str(p): digest(p) for p in files},
        native_mission=False, ground_truth_input=False, maximum_simultaneous_components=1,
        hardware_validated=False, production_promoted=False))
    save(output/'state.json', dict(status='prepared', jobs=jobs))


def run(output):
    with (output/'runner.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        config = json.loads((output/'configuration.json').read_text())
        state = json.loads((output/'state.json').read_text())
        state['status'] = 'running'
        for job in state['jobs']:
            if job['status'] == 'completed':
                continue
            directory = Path(job['output'])
            inputs = json.loads((directory/'configuration.json').read_text())
            job['status'] = 'running'
            save(output/'state.json', state)
            for variant in inputs['variants']:
                verify(config['source_sha256'])
                result_file = directory/(variant+'.result.json')
                if result_file.exists():
                    if json.loads(result_file.read_text()).get('status') == 'completed':
                        continue
                    raise ValueError('Preserve failed or interrupted arm')
                env = dict(os.environ, ROS_DOMAIN_ID='226', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                    AMENT_PREFIX_PATH=inputs['solver_prefix']+':/opt/ros/jazzy',
                    OSP_REPLAY_SOLVER='osp_solver_budget::CeresSolver', OSP_REPLAY_SOLVER_THREADS='2')
                report = dict(status='running', started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
                save(result_file, report)
                with (directory/(variant+'.log')).open('x') as stream:
                    process = subprocess.run([inputs['binary'], inputs['data'], inputs['posegraph'],
                        str(directory/variant), '1000000000', variant], env=env, stdout=stream, stderr=stream, timeout=1800)
                report.update(status='completed' if process.returncode == 0 else 'failed', returncode=process.returncode)
                save(result_file, report)
                if process.returncode:
                    raise RuntimeError('Component failed; preserve its evidence')
            job['status'] = 'completed'
            save(output/'state.json', state)
        state['status'] = 'completed'
        save(output/'state.json', state)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    for name in ['campaign', 'executable', 'solver-prefix', 'original-u-replay']:
        p.add_argument('--'+name, type=Path)
    p.add_argument('--run-prepared', action='store_true')
    a = p.parse_args()
    if a.run_prepared:
        run(a.output.resolve())
    else:
        prepare(a.campaign.resolve(), a.executable.resolve(), a.solver_prefix.resolve(),
                a.original_u_replay.resolve(), a.output.resolve())
