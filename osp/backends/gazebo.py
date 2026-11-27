"""Gazebo backend through the archived team_sim tooling.

``legacy/team-sim.sh run`` owns the simulator: it takes a per-domain slot,
enforces the host limit of two concurrent Gazebo benchmarks, keeps the
independent judge, and writes a trial directory. This backend only builds that
command, runs it when asked, and imports the trial into an ``EpisodeRecord``.
"""
import json
import os
import re
import subprocess
from pathlib import Path

from osp.paths import LEGACY
from osp.policies.team_sim import TeamSimPolicy
from osp.records import EpisodeRecord
from osp.registry import BACKENDS
from osp.scenario import load_team_scene

HOST_LIMIT = 2   # AGENTS.md: at most two native Gazebo benchmarks on this host
TEAM_SIM = LEGACY / 'team-sim.sh'
# team_sim judge end reasons: finished (current and older protocol) or timed out.
COMPLETED_REASONS = {'all_registered_and_within_1.5m': True, 'all_targets_found': True,
                     'common_time_limit': False}


def prepare_command(policy):
    return ['bash', str(TEAM_SIM), 'setup', str(policy.workspace), '--policy', policy.team_policy]


def import_trial(run_dir):
    """Read a completed team_sim trial. Returns (scenario, record), or None if incomplete."""
    run_dir = Path(run_dir)
    source = next((run_dir / n for n in ('audit.json', 'metrics.json') if (run_dir / n).is_file()), None)
    if source is None or (run_dir / 'interruption.json').exists() or not (run_dir / 'scene.json').exists():
        return None
    m = json.loads(source.read_text())
    protocol = json.loads((run_dir / 'protocol.json').read_text()) if (run_dir / 'protocol.json').exists() else {}
    scenario = load_team_scene(run_dir / 'scene.json')
    workspace = protocol.get('workspace')
    record = EpisodeRecord(
        scenario_id=scenario.id, backend='gazebo',
        policy=Path(workspace).name if workspace else 'unknown',
        duration_s=float(protocol.get('duration_sim_s', 600.)),
        seconds=m.get('elapsed_sim_s'), completed=COMPLETED_REASONS.get(m.get('reason')), collisions=m.get('bumper_events'),
        reported_ids=m.get('final_ids'),
        approach_m=m.get('min_ground_truth_m'),
        # The judge certifies approach only after the marker is registered.
        approach_witnessed=[int(i) for i in m.get('registered_and_within_s', {})],
        position_error_m=m.get('marker_position_errors_m'),
        extra_checks={'policy_sensor_proof': m.get('policy_sensor_proof_complete')},
        backend_success=m.get('strict_success'),
        metrics={k: m.get(k) for k in ('reason', 'path_length_m', 'stationary_time_s', 'unexpected_ids',
                                       'minimum_valid_scan_range_m', 'elapsed_wall_s')},
        metadata=dict(run_dir=str(run_dir), seed=protocol.get('seed'), workspace=workspace,
                      concurrency=protocol.get('concurrency'), render_backend=protocol.get('render_backend'),
                      fixture_mode=protocol.get('fixture_mode'), repository_commit=protocol.get('repository_commit')))
    return scenario, record


@BACKENDS.register('gazebo')
class GazeboBackend:
    name = 'gazebo'
    accepts = ('team_sim_seed',)

    def __init__(self, settings):
        self.settings = dict(settings)
        parallel = self.settings.get('parallel', 1)
        if parallel not in range(1, HOST_LIMIT + 1):
            raise ValueError(f'gazebo parallel must be 1..{HOST_LIMIT} on this host (got {parallel})')
        self.parallel_limit = parallel

    def command(self, scenario, policy, mission):
        return ['bash', str(TEAM_SIM), 'run', str(policy.workspace), scenario.id,
                '--duration', f'{mission.duration_s:g}']

    def environment(self):
        env = dict(os.environ, TEAM_SIM_RENDER=str(self.settings.get('render', 'system')))
        if self.parallel_limit > 1:
            env['TEAM_SIM_PARALLEL'] = str(self.parallel_limit)
        return env

    def describe(self, scenario, policy, mission):
        return ' '.join(self.command(scenario, policy, mission))

    def run_episode(self, scenario, policy, mission, log_path=None):
        if not isinstance(policy, TeamSimPolicy):
            raise TypeError(f'gazebo needs a team_sim policy, got {type(policy).__name__}')
        if not (policy.workspace / 'install/setup.bash').is_file():
            raise FileNotFoundError(f'{policy.workspace} is not a prepared workspace; run `osp prepare` first')
        policy.prepare(mission)
        command = self.command(scenario, policy, mission)
        proc = subprocess.run(command, env=self.environment(), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=self.settings.get('timeout_s', 3600))
        if log_path:
            Path(log_path).write_text(proc.stdout)
        done = re.findall(r'^COMPLETED (.+)$', proc.stdout, re.M)
        if proc.returncode or not done:
            raise RuntimeError(f'team_sim run failed (exit {proc.returncode}); see {log_path or "its output"}')
        imported = import_trial(done[-1].strip())
        if imported is None:
            raise RuntimeError(f'team_sim trial {done[-1]} is incomplete')
        truth_scenario, record = imported
        record.policy = policy.name
        return record, truth_scenario
