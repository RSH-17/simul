import json

import pytest

import official_integrated_probe as probe
import run_official_integrated_comparison as runner


EXPECTED = dict(do_loop_closing=True, loop_match_minimum_response_coarse=.55,
                loop_match_minimum_response_fine=.65, loop_match_maximum_variance_coarse=1.)


@pytest.mark.parametrize('wrong', ['do_loop_closing', 'loop_match_minimum_response_fine', 'missing'])
def test_gate_rejects_disabled_missing_or_unapplied_settings(wrong):
    observed = dict(EXPECTED)
    if wrong == 'missing':
        del observed['loop_match_minimum_response_coarse']
    else:
        observed[wrong] = False if wrong == 'do_loop_closing' else .45
    assert not probe.loop_gate(dict(parameters={'slam_toolbox': observed}), EXPECTED)['passed']
    assert probe.loop_gate(dict(parameters={'slam_toolbox': EXPECTED}), EXPECTED)['passed']


def test_no_native_dispatch_before_departure_prerequisite(tmp_path, monkeypatch):
    prior = tmp_path/'prior'
    prior.mkdir()
    (prior/'configuration.json').write_text('{}')
    runner.screen.base.save(prior/'state.json', dict(status='running', jobs=[dict(number=i, status='running') for i in [0, 1]]))
    config = dict(prerequisite=str(prior), prerequisite_source_sha256=runner.screen.base.digest(prior/'configuration.json'))
    runner.screen.base.save(tmp_path/'configuration.json', config)
    runner.screen.base.save(tmp_path/'state.json', dict(status='prepared', jobs=[]))
    monkeypatch.setattr(runner.screen, 'verify', lambda _: None)
    monkeypatch.setattr(runner.subprocess, 'run', lambda *a, **kw: pytest.fail('Launched before departure qualification'))
    runner.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status'] == 'waiting_for_verified_departure'


def test_integrated_runtime_failure_does_not_promote_native_success(tmp_path, monkeypatch):
    job = dict(output=str(tmp_path), **{k: True for k in runner.GATES})
    job['loop_settings_gate'] = False
    monkeypatch.setattr(runner.full, 'collect', lambda j: j.update(outcome=dict(success=True, infrastructure_failure=False)))
    runner.collect(job)
    assert job['raw_outcome_before_integrated_gate']['success'] is True
    assert job['outcome']['success'] is False and job['outcome']['infrastructure_failure'] is True


def test_two_external_native_jobs_prevent_a_third(tmp_path, monkeypatch):
    prior = tmp_path/'prior'
    prior.mkdir()
    (prior/'configuration.json').write_text('{}')
    runner.screen.base.save(tmp_path/'configuration.json', dict(prerequisite=str(prior),
        prerequisite_source_sha256=runner.screen.base.digest(prior/'configuration.json'), domains=[230, 231]))
    runner.screen.base.save(tmp_path/'state.json', dict(jobs=[dict(status='prepared', output=str(tmp_path/'new'))]))
    monkeypatch.setattr(runner, 'prerequisite_report', lambda _: dict(passed=True))
    monkeypatch.setattr(runner.screen, 'verify', lambda _: None)
    monkeypatch.setattr(runner.screen.base, 'service_active', lambda _: False)
    monkeypatch.setattr(runner.screen.base, 'occupied_slots', lambda _: 2)
    monkeypatch.setattr(runner.subprocess, 'run', lambda *a, **kw: pytest.fail('Third native dispatched'))
    def stop(_):
        raise InterruptedError()
    monkeypatch.setattr(runner.time, 'sleep', stop)
    with pytest.raises(InterruptedError):
        runner.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status'] == 'waiting_for_native_slots'
