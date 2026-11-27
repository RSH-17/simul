import json

import pytest

import run_official_camera_view_screen as runner


def test_cannot_dispatch_beside_active_matrix(monkeypatch,tmp_path):
    monkeypatch.setattr(runner.local.screen.base,'service_active',lambda _:True)
    monkeypatch.setattr(runner.matrix,'run',lambda _:pytest.fail('Competing dispatch'))
    with pytest.raises(RuntimeError,match='dispatcher must be held'):
        runner.run(tmp_path)


@pytest.mark.parametrize('status',['completed','needs_infrastructure_repair'])
def test_resume_requires_terminal_completed_comparison(monkeypatch,tmp_path,status):
    monkeypatch.setattr(runner.local.screen.base,'service_active',lambda _:False)
    monkeypatch.setattr(runner.matrix,'run',lambda _:None)
    calls=[];monkeypatch.setattr(runner.subprocess,'run',lambda args,**_:calls.append(args))
    (tmp_path/'state.json').write_text(json.dumps({'status':status}))
    runner.run(tmp_path)
    assert calls==([['systemctl','--user','start',runner.MATRIX]] if status=='completed' else [])
