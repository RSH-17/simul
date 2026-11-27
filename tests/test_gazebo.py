import json
import subprocess

import pytest

from osp.app import cli
from osp.core import config
from osp.scenarios import catalog
from osp.backends.gazebo import GazeboBackend, import_trial
from osp.core.paths import team_sim_home
from osp.policies.base import build
from osp.core.registry import load_builtins


def test_command_goes_through_team_sim():
    cfg = config.load('gazebo', 'frontier_tour', ['mission.duration_s=120'])
    load_builtins()
    backend = GazeboBackend(cfg['backend']['settings'])
    scenario = catalog.load('exam/dev')[0]
    cmd = backend.command(scenario, build(cfg), scenario.mission(3, 120))
    assert cmd[1].endswith('legacy/team-sim.sh') and cmd[2:4] == ['run', str(build(cfg).workspace)]
    assert cmd[4:] == ['112601', '--duration', '120']


@pytest.mark.parametrize('parallel', [0, 3])
def test_host_limit_of_two(parallel):
    with pytest.raises(ValueError, match='1..2'):
        GazeboBackend(dict(parallel=parallel))


def test_parallel_two_is_declared_to_team_sim():
    assert GazeboBackend(dict(parallel=2)).environment()['TEAM_SIM_PARALLEL'] == '2'


def test_gazebo_and_prepare_are_dry_runs_by_default(monkeypatch, tmp_path, capsys):
    real_popen = subprocess.Popen

    def refuse(command, *a, **k):
        if command[0] == 'git':                     # run metadata only
            return real_popen(command, *a, **k)
        raise AssertionError(f'launched {command} without --execute')
    monkeypatch.setattr(subprocess, 'Popen', refuse)
    assert cli.main(['run', '--backend', 'gazebo', '--policy', 'far_wall_slot', '--scenarios', 'exam/dev',
                     '--repeats', '2', '--out', str(tmp_path / 'r')]) == 0
    assert cli.main(['prepare', '--policy', 'far_wall_slot']) == 0
    out = capsys.readouterr().out
    assert out.count('PLAN bash') == 11 and 'Dry run' in out
    assert len((tmp_path / 'r/plan.json').read_text().splitlines()) == 10
    assert not (tmp_path / 'r/episodes.jsonl').exists()


def test_wrong_scenario_kind_is_rejected(tmp_path):
    assert cli.main(['run', '--backend', 'gazebo', '--policy', 'mixed_v151', '--scenarios', 'robust/test',
                     '--out', str(tmp_path / 'r')]) == 2


RESULTS = team_sim_home() / 'results'
TRIALS = sorted(p for p in RESULTS.iterdir() if p.is_dir()) if RESULTS.is_dir() else []


@pytest.mark.skipif(not TRIALS, reason='no local team_sim results')
def test_rescoring_real_trials_matches_the_legacy_strict_judge():
    from osp.evaluation import evaluator
    base = config.base()
    compared = 0
    for run_dir in TRIALS:
        imported = import_trial(run_dir)
        if imported is None:
            continue
        scenario, record = imported
        verdict = evaluator.evaluate(record, scenario, base['criteria'], base['mission']['marker_count'])
        assert (verdict['status'] == 'success') == bool(record.backend_success), run_dir.name
        assert verdict['status'] != 'unverified', run_dir.name
        compared += 1
    assert compared >= 1


def test_incomplete_trial_is_skipped(tmp_path):
    (tmp_path / 'metrics.json').write_text(json.dumps({}))
    (tmp_path / 'interruption.json').write_text('{}')
    assert import_trial(tmp_path) is None
