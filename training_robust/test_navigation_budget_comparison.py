import json

import pytest

import run_official_navigation_budget_comparison as runner


@pytest.mark.parametrize('missing',['limit_s','resumes','active_s','clock_fault'])
def test_unapplied_or_incomplete_live_budget_does_not_pass(missing):
    observed=dict(limit_s=65.,resumes=0,active_s=12.,assigned=True,active=True,clock_fault=False)
    assert runner.budget_gate(dict(navigation_budget=observed),True)['passed']
    del observed[missing]
    assert not runner.budget_gate(dict(navigation_budget=observed),True)['passed']
    assert runner.budget_gate({},False)['passed']
    assert not runner.budget_gate(dict(navigation_budget=observed),False)['passed']


@pytest.mark.parametrize('state_status,job_status,active',[('running','running',True),('completed','done',True),('running','done',False)])
def test_old_coordinator_or_job_prevents_handoff(tmp_path,monkeypatch,state_status,job_status,active):
    prior=tmp_path/'prior';prior.mkdir();(prior/'configuration.json').write_text('{}')
    (prior/'state.json').write_text(json.dumps(dict(status=state_status,coordinator_unit='old',
        jobs=[dict(status=job_status,unit='old-job')])))
    config=dict(waits_for_campaign=str(prior),waits_for_configuration_sha256=runner.screen.base.digest(prior/'configuration.json'),wait_limit_wall_s=43200.)
    (tmp_path/'configuration.json').write_text(json.dumps(config))
    monkeypatch.setattr(runner.screen.base,'service_active',lambda _:active)
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner.integrated,'run',lambda _:pytest.fail('Premature native dispatch'))
    def stop(_):raise InterruptedError()
    monkeypatch.setattr(runner.time,'sleep',stop)
    with pytest.raises(InterruptedError):runner.run(tmp_path)


def test_terminal_old_campaign_uses_existing_resource_limited_dispatch_and_extra_gate(tmp_path,monkeypatch):
    (tmp_path/'configuration.json').write_text('{}')
    monkeypatch.setattr(runner,'original_finished',lambda _:True)
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    original=runner.integrated.observe;called=[]
    def dispatch(output):
        called.append(output)
        assert 'navigation_budget_gate' in runner.integrated.GATES
        assert runner.integrated.UNIT in runner.integrated.BLOCKERS
        assert runner.integrated.observe is runner.observe
    monkeypatch.setattr(runner.integrated,'run',dispatch)
    runner.run(tmp_path)
    assert called==[tmp_path] and runner.integrated.observe is original


def test_extra_odometry_change_cannot_enter_isolated_budget_trial(tmp_path,monkeypatch):
    baseline,candidate=tmp_path/'baseline',tmp_path/'candidate'
    def hashes(path):
        if path==baseline:return {'osp_mission/mission.py':'old'}
        return {k:'new' for k in [*runner.EXPECTED_DELTA,'config/odom.yaml']}
    monkeypatch.setattr(runner.screen.base,'source_hashes',hashes)
    with pytest.raises(ValueError,match='isolated navigation budget'):
        runner.prepare(tmp_path/'out',baseline,candidate,tmp_path/'old',tmp_path/'policy')
    assert not (tmp_path/'out').exists()


@pytest.mark.parametrize('invalid',[float('inf'),float('nan'),True,-1.])
def test_invalid_elapsed_time_cannot_verify_the_live_budget(invalid):
    observed=dict(limit_s=65.,resumes=1,active_s=invalid,assigned=True,active=True,clock_fault=False)
    assert not runner.budget_gate(dict(navigation_budget=observed),True)['passed']
