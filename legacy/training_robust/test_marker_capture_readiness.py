import json

import pytest

import capture_marker_range_for_job_v2 as watcher


@pytest.mark.parametrize('not_ready',['live','platform','startup',None])
def test_capture_waits_for_sensor_warmup_without_restarting_the_native_job(tmp_path,monkeypatch,not_ready):
    monkeypatch.setattr(watcher,'native_pid',lambda *args:12345)
    job=dict(number=3,status='running',domain=231,unit='osp-test.service',output=str(tmp_path),scenario={'seed':5})
    state={'jobs':[job]};episode=tmp_path/'episodes/5';episode.mkdir(parents=True)
    assert watcher.select_job(state,3)[1] is None
    (episode/'live_status.json').write_text(json.dumps({'status':{'state':'waiting' if not_ready=='live' else 'running'}}))
    (episode/'tb4_platform_observation.json').write_text(json.dumps({'passed':not_ready!='platform'}))
    (episode/'startup_guard.json').write_text(json.dumps({'success':not_ready!='startup'}))
    assert watcher.select_job(state,3)[1]==(12345 if not_ready is None else None)
