import json

import pytest

import run_official_slam_gate_study as study


def test_changed_scan_input_prevents_mapper_execution(tmp_path, monkeypatch):
    data = tmp_path/'map.data'
    data.write_bytes(b'original')
    expected = study.digest(data)
    data.write_bytes(b'changed')
    case = tmp_path/'case'
    case.mkdir()
    study.save(case/'configuration.json', dict(variants=['baseline']))
    study.save(tmp_path/'configuration.json', dict(source_sha256={str(data): expected}))
    study.save(tmp_path/'state.json', dict(status='prepared', jobs=[dict(status='prepared', output=str(case))]))
    monkeypatch.setattr(study.subprocess, 'run', lambda *a, **kw: pytest.fail('Changed input reached mapper'))
    with pytest.raises(ValueError, match='Frozen component input changed'):
        study.run(tmp_path)


def test_completed_study_preserves_results_without_rerunning(tmp_path, monkeypatch):
    study.save(tmp_path/'configuration.json', dict(source_sha256={}))
    study.save(tmp_path/'state.json', dict(status='completed', jobs=[dict(status='completed')]))
    monkeypatch.setattr(study.subprocess, 'run', lambda *a, **kw: pytest.fail('Repeated a completed mapper'))
    study.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status'] == 'completed'
