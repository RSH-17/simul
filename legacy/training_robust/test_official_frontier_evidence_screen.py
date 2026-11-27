import json
from pathlib import Path

import pytest

import run_official_frontier_evidence_screen as runner


def test_candidate_preparation_keeps_inputs_limits_and_unique_services(tmp_path, monkeypatch):
    old, new, prior = [tmp_path/name for name in ['old', 'new', 'prior']]
    for root in [old, new]:
        (root/'src/osp_mission').mkdir(parents=True)
        (root/'install').mkdir()
        (root/'install/setup.bash').write_text('')
        (root/'src/osp_mission/body.yaml').write_text('unchanged')
    for name in runner.DELTA:
        p = new/'src/osp_mission'/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('isolated frontier evidence')
    prior.mkdir()
    (prior/'configuration.json').write_text('{"driver_hashes":{}}')
    monkeypatch.setattr(runner.protocol.screen, 'verify', lambda _: None)
    jobs = []
    for i, profile in enumerate(['sideways_small', 'narrow']):
        directory = prior/str(i)
        directory.mkdir()
        (directory/'policy.json').write_text('{"parameters":[1,2,3,4,5]}')
        scene = directory/'world.yaml'
        scene.write_text(f'seed: {i}')
        scenario = dict(seed=i, file=str(scene), constraint_profile=profile)
        (directory/'manifest.json').write_text(json.dumps(dict(duration=90., scenarios=[scenario])))
        jobs.append(dict(status='done', workspace=str(old), output=str(directory), scenario=scenario,
            outcome=dict(success=False, infrastructure_failure=False, contacts=0, start_screen_passed=True)))
    (prior/'state.json').write_text(json.dumps(dict(jobs=jobs)))
    original_unit, original_delta = runner.protocol.UNIT, runner.protocol.DELTA
    out = tmp_path/'comparison'
    runner.prepare(out, new, prior, tmp_path/'frontier')
    state = json.loads((out/'state.json').read_text())
    config = json.loads((out/'configuration.json').read_text())
    assert state['coordinator_unit'] == runner.UNIT
    assert config['native_limit'] == 2 and config['holdout_consumed'] is False
    assert str(Path(runner.__file__).resolve()) in config['driver_hashes']
    assert runner.protocol.UNIT == original_unit and runner.protocol.DELTA == original_delta
    for original, job in zip(jobs, state['jobs']):
        assert job['unit'].startswith('osp-official-frontier-evidence-v1-')
        for name in ['manifest.json', 'policy.json']:
            assert (Path(original['output'])/name).read_bytes() == (Path(job['output'])/name).read_bytes()
    (new/'src/osp_mission/body.yaml').write_text('changed')
    with pytest.raises(ValueError):
        runner.prepare(tmp_path/'invalid', new, prior, tmp_path/'frontier')
    assert runner.protocol.UNIT == original_unit and runner.protocol.DELTA == original_delta
