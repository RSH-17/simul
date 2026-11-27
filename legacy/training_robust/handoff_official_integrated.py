"""Wait for v16 departure results, drain older jobs, then start the prepared pair.

Only dispatchers are stopped. Existing native service groups are never stopped
or restarted. Original queued jobs and runtime sources remain preserved.
"""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time

import run_official_integrated_comparison as integrated

screen = integrated.screen
full = integrated.full


def ready(prerequisite, prerequisite_unit):
    # The old supervisor can still resume another dispatcher after writing its
    # completion file. Wait for its actual service to exit before handoff.
    if screen.base.service_active(prerequisite_unit):
        return False
    return json.loads((prerequisite/'state.json').read_text())['status'] == 'completed'


def drain_campaign(path, is_full):
    with (path/'coordinator.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        screen.verify(json.loads((path/'configuration.json').read_text()))
        state = json.loads((path/'state.json').read_text())
        for job in state['jobs']:
            if job['status'] == 'prepared' and (Path(job['output'])/'service.json').exists():
                job['status'] = 'running'
            if job['status'] != 'running':
                continue
            if is_full:
                if screen.base.service_active(job['unit']):
                    full.observe(job)
                elif not screen.base.benchmark_output_live(job['output']):
                    full.collect(job)
            else:
                # Match the original local-grid matrix's observer/collector,
                # including its navigation-grid and independent pose audits.
                saved = (screen.probe, screen.configuration_gate, screen.GATES)
                try:
                    screen.probe, screen.configuration_gate = full.grid.probe, full.grid.configuration_gate
                    screen.GATES = tuple(dict.fromkeys([*saved[2], 'navigation_grid_gate']))
                    if screen.base.service_active(job['unit']):
                        screen.observe(job)
                        p = Path(job['output'])/'runtime_configuration.json'
                        if p.exists():
                            result = full.grid.navigation_grid_gate(json.loads(p.read_text()))
                            job['navigation_grid_gate'] = result['passed']
                            screen.base.save(p.parent/'navigation_grid_check.json', result)
                    elif not screen.base.benchmark_output_live(job['output']):
                        screen.collect(job)
                        try:
                            episode = Path(job['output'])/'episodes'/str(job['scenario']['seed'])
                            result = full.audit_pose(episode)
                            screen.base.save(Path(job['output'])/'pose_audit.json', result)
                            job['localization_audit'] = result['pose_errors']
                        except (OSError, ValueError, KeyError, IndexError) as exc:
                            job['localization_audit_error'] = repr(exc)
                finally:
                    screen.probe, screen.configuration_gate, screen.GATES = saved
        state['status'] = ('completed' if all(j['status'] == 'done' for j in state['jobs'])
                           else 'held_for_integrated_source_comparison')
        screen.base.save(path/'state.json', state)
        return [j for j in state['jobs'] if j['status'] == 'running']


def run(output):
    with (output/'handoff.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        config = json.loads((output/'handoff_configuration.json').read_text())
        prerequisite = Path(config['prerequisite'])
        def verify():
            for path, expected in config['source_sha256'].items():
                if screen.base.digest(Path(path)) != expected:
                    raise ValueError('Frozen handoff changed: '+path)
            screen.verify(json.loads((output/'configuration.json').read_text()))
        while not ready(prerequisite, config['prerequisite_unit']):
            verify()
            screen.base.save(output/'handoff_state.json', dict(status='waiting_for_departure_service_completion'))
            time.sleep(5.)
        verify()
        report = integrated.prerequisite_report(prerequisite)
        screen.base.save(output/'prerequisite_check.json', report)
        before = screen.base.native_processes()
        for unit in [full.UNIT, full.MATRIX]:
            subprocess.run(['systemctl', '--user', 'stop', unit], check=True)
        screen.base.save(output/'handoff_state.json', dict(status='draining_preserved_native_jobs',
            stopped_dispatchers=[full.UNIT, full.MATRIX], native_pids_before=before,
            native_pids_after=screen.base.native_processes(), native_stop_requested=False))
        while True:
            verify()
            remaining = drain_campaign(Path(config['original_full']), True)
            remaining += drain_campaign(Path(config['original_matrix']), False)
            if not remaining and not screen.base.native_processes():
                break
            time.sleep(5.)
        if not report['passed']:
            screen.base.save(output/'handoff_state.json', dict(status='departure_repair_required',
                prerequisite=report, native_stop_requested=False, original_queued_jobs_preserved=True))
            return
        subprocess.run(['systemctl', '--user', 'start', integrated.UNIT], check=True)
        screen.base.save(output/'handoff_state.json', dict(status='integrated_dispatcher_started',
            unit=integrated.UNIT, native_stop_requested=False, original_queued_jobs_preserved=True))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run(a.output.resolve())
