import json

import pytest

import handoff_official_integrated as handoff


def test_completion_file_does_not_override_live_prerequisite_service(tmp_path, monkeypatch):
    (tmp_path/'state.json').write_text(json.dumps(dict(status='completed')))
    monkeypatch.setattr(handoff.screen.base, 'service_active', lambda _: True)
    assert not handoff.ready(tmp_path, 'original.service')


def test_drain_never_collects_or_stops_live_original_native(tmp_path, monkeypatch):
    (tmp_path/'configuration.json').write_text('{}')
    (tmp_path/'state.json').write_text(json.dumps(dict(jobs=[
        dict(status='running', unit='native.service', output=str(tmp_path/'native')),
        dict(status='prepared', output=str(tmp_path/'queued'))])))
    monkeypatch.setattr(handoff.screen, 'verify', lambda _: None)
    monkeypatch.setattr(handoff.screen.base, 'service_active', lambda _: True)
    observed = []
    monkeypatch.setattr(handoff.full, 'observe', lambda j: observed.append(j['unit']))
    monkeypatch.setattr(handoff.full, 'collect', lambda _: pytest.fail('Collected live native job'))
    monkeypatch.setattr(handoff.subprocess, 'run', lambda *a, **kw: pytest.fail('Issued control command while draining'))
    live = handoff.drain_campaign(tmp_path, True)
    assert observed == ['native.service'] and len(live) == 1
    assert json.loads((tmp_path/'state.json').read_text())['jobs'][1]['status'] == 'prepared'
