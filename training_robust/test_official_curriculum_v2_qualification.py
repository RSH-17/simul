import json
from pathlib import Path

import pytest

import launch_official_tb4_curriculum_v2 as launcher


def test_only_finished_physical_escape_with_unchanged_runtime_qualifies(tmp_path,monkeypatch):
    workspace=tmp_path/'runtime';(workspace/'src/osp_mission').mkdir(parents=True)
    source=workspace/'src/osp_mission/source.py';source.write_text('original\n')
    screen=tmp_path/'screen';screen.mkdir()
    gates=('runtime_configuration_gate','velocity_profile_gate','scan_adapter_gate',
           'floor_sensor_gate','drive_path_gate','official_description_gate','slam_transport_gate')
    state=dict(jobs=[dict(workspace=str(workspace),**{k:True for k in gates})])
    def save(name,value): (screen/name).write_text(json.dumps(value))
    save('state.json',state);save('report.json',dict(complete=True))
    save('source_hashes.json',launcher.runner.source_hashes(workspace))
    result=dict(completed=True,contacts=0,observed_exit_seconds=20.)
    monkeypatch.setattr(launcher,'audit_escape',lambda _:result)
    assert launcher.qualify(workspace,screen)['hardware_validated'] is False
    for key,value in [('completed',False),('contacts',1),('observed_exit_seconds',None)]:
        original=result[key];result[key]=value
        with pytest.raises(ValueError):launcher.qualify(workspace,screen)
        result[key]=original
    state['jobs'][0]['scan_adapter_gate']=False;save('state.json',state)
    with pytest.raises(ValueError):launcher.qualify(workspace,screen)
    state['jobs'][0]['scan_adapter_gate']=True;save('state.json',state)
    source.write_text('changed\n')
    with pytest.raises(ValueError):launcher.qualify(workspace,screen)
