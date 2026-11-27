import json

import pytest

import run_official_frontier_comparison as runner


def runtime(path,candidate=False):
    package=path/'src/osp_mission';(package/'osp_mission').mkdir(parents=True)
    (package/'test').mkdir();(path/'install').mkdir();(path/'install/setup.bash').write_text('')
    (package/'osp_mission/edge_policy.py').write_text('new' if candidate else 'old')
    (package/'body.yaml').write_text('unchanged body')
    if candidate:
        (package/'osp_mission/frontier_support.py').write_text('gain only')
        (package/'test/test_frontier_support.py').write_text('test gain')
    return path


def test_pair_rejects_unrelated_robot_changes(tmp_path):
    a=runtime(tmp_path/'old');b=runtime(tmp_path/'new',True)
    assert len(runner.compare_sources(a,b))==2
    (b/'src/osp_mission/body.yaml').write_text('smaller robot')
    with pytest.raises(ValueError,match='differs beyond'):runner.compare_sources(a,b)


def test_full_task_gate_does_not_accept_legacy_ninety_seconds(monkeypatch):
    monkeypatch.setattr(runner.grid,'configuration_gate',lambda r,p,expected_duration:r['duration']==expected_duration)
    assert not runner.full_configuration_gate({'duration':90},None,expected_duration=90)
    assert runner.full_configuration_gate({'duration':600},None,expected_duration=90)


def test_preparation_preserves_paired_worlds_weights_and_frozen_inputs(tmp_path,monkeypatch):
    baseline=runtime(tmp_path/'old');candidate=runtime(tmp_path/'new',True)
    comparison=tmp_path/'comparison';comparison.mkdir()
    (comparison/'policy.json').write_text('{"planner":"edge","parameters":[1,2,3,4,5]}')
    monkeypatch.setattr(runner,'qualify',lambda *a:{'screen':'qualified parent only'})
    matrix=tmp_path/'matrix';matrix.mkdir();(matrix/'configuration.json').write_text('{"driver_hashes":{}}')
    jobs=[None]*18
    for index,shape in [(0,'c_shape'),(9,'u_shape'),(17,'zigzag')]:
        scene=matrix/f'{index}.yaml';scene.write_text(f'seed: {index}\n')
        directory=matrix/f'job_{index}';directory.mkdir()
        scenario=dict(seed=index,file=str(scene),sha256=runner.screen.base.digest(scene),arena={'shape':shape})
        (directory/'manifest.json').write_text(json.dumps({'duration':90,'scenarios':[scenario]}))
        jobs[index]=dict(output=str(directory),scenario=scenario)
    (matrix/'state.json').write_text(json.dumps({'jobs':jobs}))
    output=tmp_path/'paired';runner.prepare(output,baseline,candidate,comparison,matrix)
    state=json.loads((output/'state.json').read_text());assert len(state['jobs'])==6
    for a,b in zip(state['jobs'][::2],state['jobs'][1::2]):
        assert a['scenario']==b['scenario']
        from pathlib import Path
        pa,pb=Path(a['output']),Path(b['output'])
        assert (pa/'policy.json').read_bytes()==(pb/'policy.json').read_bytes()==(comparison/'policy.json').read_bytes()
        assert json.loads((pa/'manifest.json').read_text())['duration']==600
    config=json.loads((output/'configuration.json').read_text());runner.screen.verify(config)
    (matrix/'0.yaml').write_text('changed world')
    with pytest.raises(RuntimeError,match='changed'):runner.screen.verify(config)


def test_observer_interference_cannot_become_a_success(tmp_path,monkeypatch):
    job=dict(output=str(tmp_path),scenario={'seed':1},**{k:True for k in runner.GATES})
    monkeypatch.setattr(runner.screen.base,'collect',lambda j:j.update(status='done',outcome={'success':True,'infrastructure_failure':False}))
    monkeypatch.setattr(runner.screen.base,'candidate_profiles',lambda _:True)
    monkeypatch.setattr(runner,'audit_pose',lambda _:dict(pose_errors={}))
    monkeypatch.setattr(runner.screen,'escape_audit',lambda _:dict(completed=True))
    (tmp_path/'environment_invalidation.json').write_text('{"policy_training_sample_valid":false}')
    runner.collect(job)
    assert job['raw_outcome_before_environment_invalidation']['success'] is True
    assert job['outcome']['success'] is False and job['outcome']['infrastructure_failure'] is True


def test_no_third_native_job_is_dispatched(tmp_path,monkeypatch):
    (tmp_path/'configuration.json').write_text(json.dumps({'domains':[230,231]}))
    jobs=[{'status':'prepared','output':str(tmp_path/'job'),'scenario':{'arena':{'shape':'c_shape'}}}]
    (tmp_path/'state.json').write_text(json.dumps({'jobs':jobs}))
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner.screen.base,'service_active',lambda _:False)
    monkeypatch.setattr(runner.screen.base,'native_processes',lambda:[])
    monkeypatch.setattr(runner.screen.base,'occupied_slots',lambda _:2)
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**kw:pytest.fail('Third benchmark launched'))
    def end_iteration(_):raise InterruptedError('test iteration ended')
    monkeypatch.setattr(runner.time,'sleep',end_iteration)
    with pytest.raises(InterruptedError):runner.run(tmp_path)
    assert json.loads((tmp_path/'state.json').read_text())['status']=='waiting_for_native_slots'
