import json
import pytest
import run_rolling_goal_comparison as runner


@pytest.mark.parametrize('state,unit_active',[('running',True),('running',False),('completed',True)])
def test_no_new_native_before_original_trial_and_coordinator_finish(tmp_path,monkeypatch,state,unit_active):
    old=tmp_path/'original';old.mkdir();(old/'configuration.json').write_text('{}')
    runner.screen.base.save(old/'state.json',dict(status=state,coordinator_unit='original.service',
        jobs=[dict(status='done',unit='original-job.service')]))
    config=dict(waits_for_campaign=str(old),waits_for_configuration_sha256=runner.screen.base.digest(old/'configuration.json'),wait_limit_wall_s=43200.)
    runner.screen.base.save(tmp_path/'configuration.json',config)
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner,'service_active',lambda _:unit_active)
    monkeypatch.setattr(runner.integrated,'run',lambda _:pytest.fail('New native launched before handoff'))
    def stop(_):raise InterruptedError()
    monkeypatch.setattr(runner.time,'sleep',stop)
    with pytest.raises(InterruptedError):runner.run(tmp_path)


def test_terminal_original_uses_existing_two_native_dispatcher_with_rolling_gate(tmp_path,monkeypatch):
    runner.screen.base.save(tmp_path/'configuration.json',{})
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner,'original_finished',lambda _:True)
    calls=[]
    def dispatch(output):
        calls.append(output)
        assert 'rolling_goal_gate' in runner.integrated.GATES
        assert 'osp-official-frontier-coverage-v1.service' in runner.integrated.BLOCKERS
    monkeypatch.setattr(runner.integrated,'run',dispatch)
    runner.run(tmp_path);assert calls==[tmp_path]


def test_other_policy_changes_rejected_before_pair_is_prepared(tmp_path,monkeypatch):
    baseline=tmp_path/'baseline'
    monkeypatch.setattr(runner.screen.base,'source_hashes',lambda w:{} if w==baseline else {'config/tb4_slam.yaml':'changed'})
    with pytest.raises(ValueError,match='isolated'):
        runner.prepare(tmp_path/'out',baseline,tmp_path/'new',tmp_path/'prior',tmp_path/'policy')
    assert not (tmp_path/'out').exists()
