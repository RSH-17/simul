import json
from pathlib import Path

import pytest
import run_official_dock_screen as runner


@pytest.mark.parametrize('dock_ok,contacts,expected',[(False,0,False),(True,4,False),(True,0,True)])
def test_passing_departure_requires_actual_dock_gate_and_zero_contacts(tmp_path,monkeypatch,dock_ok,contacts,expected):
    job=dict(output=str(tmp_path),scenario={'seed':1},**{k:True for k in runner.GATES});job['dock_map_gate']=dock_ok
    def original(j):
        j.update(status='done',outcome=dict(success=False,infrastructure_failure=False,contacts=contacts),
            escape_audit=dict(completed=True,contacts=contacts,observed_exit_seconds=24.))
    monkeypatch.setattr(runner.screen,'collect',original)
    monkeypatch.setattr(runner.screen.base,'candidate_profiles',lambda _:True)
    monkeypatch.setattr(runner,'audit_pose',lambda _:dict(pose_errors={}))
    runner.collect(job)
    assert job['start_screen_passed'] is expected
    assert job['outcome']['infrastructure_failure'] is (not dock_ok)
    assert job['outcome']['success'] is False


def test_draining_does_not_cancel_or_collect_live_original_native(tmp_path,monkeypatch):
    state=dict(jobs=[dict(status='running',unit='original.service',output='original')])
    (tmp_path/'state.json').write_text(json.dumps(state))
    monkeypatch.setattr(runner.screen.base,'service_active',lambda _:True)
    monkeypatch.setattr(runner.full,'collect',lambda _:pytest.fail('Collected a running benchmark'))
    runner.collect_drained_frontier(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['jobs'][0]['status']=='running'


def test_two_external_benchmarks_prevent_third_dispatch(tmp_path,monkeypatch):
    (tmp_path/'configuration.json').write_text(json.dumps({'domains':[228,229],'frontier_campaign':str(tmp_path)}))
    (tmp_path/'state.json').write_text(json.dumps({'jobs':[dict(status='prepared',output=str(tmp_path/'new'))]}))
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner.screen.base,'service_active',lambda _:False)
    monkeypatch.setattr(runner,'collect_drained_frontier',lambda _:None)
    monkeypatch.setattr(runner.screen.base,'occupied_slots',lambda _:2)
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**kw:pytest.fail('Third native dispatched'))
    def stop(_):raise InterruptedError()
    monkeypatch.setattr(runner.time,'sleep',stop)
    with pytest.raises(InterruptedError):runner.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status']=='waiting_for_native_slots'


def test_preparation_isolates_dock_delta_and_preserves_failed_scene(tmp_path,monkeypatch):
    old=tmp_path/'old';new=tmp_path/'new'
    for p in [old,new]:
        (p/'src/osp_mission').mkdir(parents=True);(p/'install').mkdir()
        (p/'install/setup.bash').write_text('');(p/'src/osp_mission/body.yaml').write_text('body remains unchanged')
    for name in runner.DELTA:
        p=new/'src/osp_mission'/name;p.parent.mkdir(exist_ok=True,parents=True);p.write_text('dock change')
    matrix=tmp_path/'matrix';matrix.mkdir();(matrix/'configuration.json').write_text('{"driver_hashes":{}}')
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    jobs=[None]*5
    for i,profile in [(0,'narrow'),(4,'sideways_small')]:
        directory=matrix/str(i);directory.mkdir();(directory/'policy.json').write_text('{"parameters":[1,2,3,4,5]}')
        scene=directory/'world.yaml';scene.write_text(f'seed: {i}')
        scenario=dict(seed=i,file=str(scene),constraint_profile=profile)
        (directory/'manifest.json').write_text(json.dumps({'duration':90.,'scenarios':[scenario]}))
        jobs[i]=dict(status='done',workspace=str(old),output=str(directory),scenario=scenario,
            outcome=dict(success=False,infrastructure_failure=False,contacts=4 if i==4 else 0))
    (matrix/'state.json').write_text(json.dumps({'jobs':jobs}))
    out=tmp_path/'comparison';runner.prepare(out,new,matrix,tmp_path/'frontier')
    result=json.loads((out/'state.json').read_text());assert len(result['jobs'])==2
    assert result['jobs'][0]['original_outcome']['contacts']==4
    assert (Path(result['jobs'][0]['output'])/'manifest.json').read_bytes()==(matrix/'4/manifest.json').read_bytes()
    (new/'src/osp_mission/body.yaml').write_text('smaller robot')
    with pytest.raises(ValueError,match='isolate'):runner.prepare(tmp_path/'wrong',new,matrix,tmp_path/'frontier')
