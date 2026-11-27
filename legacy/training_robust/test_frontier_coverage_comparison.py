import subprocess
from types import SimpleNamespace

import pytest
import run_frontier_coverage_comparison as runner


def test_missing_historical_unit_is_terminal_not_active(monkeypatch):
    monkeypatch.setattr(runner.integrated.subprocess,'run',lambda *a,**k:
        SimpleNamespace(returncode=1,stdout='LoadState=not-found\nActiveState=inactive\n',stderr=''))
    assert not runner.service_active('old.service')


def test_service_bus_failure_cannot_be_treated_as_free_slot(monkeypatch):
    monkeypatch.setattr(runner.integrated.subprocess,'run',lambda *a,**k:
        SimpleNamespace(returncode=1,stdout='',stderr='Failed to connect to bus'))
    with pytest.raises(RuntimeError):runner.service_active('live.service')


@pytest.mark.parametrize('state',['active','activating','deactivating'])
def test_live_unit_owns_its_slot(monkeypatch,state):
    monkeypatch.setattr(runner.integrated.subprocess,'run',lambda *a,**k:
        SimpleNamespace(returncode=0,stdout='LoadState=loaded\nActiveState='+state+'\n',stderr=''))
    assert runner.service_active('live.service')


def test_original_trial_dispatch_checks_host_limit_before_new_launch(tmp_path,monkeypatch):
    import json
    prior=tmp_path/'prior';prior.mkdir();(prior/'configuration.json').write_text('{}')
    runner.screen.base.save(tmp_path/'configuration.json',dict(prerequisite=str(prior),
        prerequisite_source_sha256=runner.screen.base.digest(prior/'configuration.json'),domains=[230,231]))
    runner.screen.base.save(tmp_path/'state.json',dict(jobs=[dict(status='prepared',output=str(tmp_path/'new'))]))
    monkeypatch.setattr(runner.integrated,'prerequisite_report',lambda _:dict(passed=True))
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner,'service_active',lambda _:False)
    monkeypatch.setattr(runner.screen.base,'occupied_slots',lambda _:2)
    monkeypatch.setattr(runner.integrated.subprocess,'run',lambda *a,**kw:pytest.fail('Third native launched'))
    def stop(_):raise InterruptedError()
    monkeypatch.setattr(runner.integrated.time,'sleep',stop)
    with pytest.raises(InterruptedError):runner.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status']=='waiting_for_native_slots'
