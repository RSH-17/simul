import pytest

import capture_marker_range_for_job as watcher


def state(status,domain=230):
    return {'jobs':[dict(number=2,status=status,domain=domain,unit='osp-test.service',output='/test')]}


def test_prepared_or_completed_state_never_claims_a_running_capture(monkeypatch):
    monkeypatch.setattr(watcher,'native_pid',lambda *a:pytest.fail('Inactive job was probed'))
    assert watcher.select_job(state('prepared'),2)[1] is None
    assert watcher.select_job(state('done'),2)[1] is None


def test_running_state_requires_actual_benchmark_process(monkeypatch):
    monkeypatch.setattr(watcher,'native_pid',lambda *a:None)
    assert watcher.select_job(state('running'),2)[1] is None
    monkeypatch.setattr(watcher,'native_pid',lambda *a:12345)
    assert watcher.select_job(state('running'),2)[1]==12345


@pytest.mark.parametrize('domain',[None,True,'230',233])
def test_unknown_domain_cannot_attach_to_a_different_robot(domain):
    with pytest.raises(ValueError):watcher.select_job(state('running',domain),2)
