import copy
import json
from pathlib import Path

import pytest
import yaml

import run_official_constraint_screen as screen


def test_start_pass_requires_final_contacts_input_and_bounded_exit():
    result=dict(completed=True, contacts=0, observed_exit_seconds=40.)
    assert screen.screen_outcome(result, True)
    assert not screen.screen_outcome(result, False)
    for key,value in [('completed',False),('contacts',1),('contacts',None),
                      ('observed_exit_seconds',None),('observed_exit_seconds',91.),
                      ('observed_exit_seconds',-1.)]:
        assert not screen.screen_outcome(dict(result,**{key:value}), True)


def test_preparation_never_consumes_reserved_scenes_or_leaks_truth_to_policy(tmp_path):
    matrix=tmp_path/'matrix';matrix.mkdir()
    campaign=tmp_path/'campaign';campaign.mkdir()
    rows=[]
    for i in range(18):
        path=matrix/f'{i}.yaml'
        path.write_text(yaml.safe_dump(dict(seed=i,arena=dict(shape='u_shape'),targets=[dict(id=42)])))
        rows.append(dict(split='train',file=str(path),sha256=screen.base.digest(path),seed=i,
                         physical_sha256=str(i),obstacles=12,shape='u_shape',profile=str(i)))
    rows.append(dict(split='holdout',file='/must/not/be/read',sha256='unread'))
    (matrix/'manifest.json').write_text(json.dumps(dict(episodes=rows)))
    (campaign/'configuration.json').write_text(json.dumps(dict(workspace='/frozen',
        source_hashes={},driver_hashes={},baseline_weights=[1,2,3,4,5])))
    output=tmp_path/'screen';screen.prepare(output,matrix,campaign)
    state=json.loads((output/'state.json').read_text())
    assert len(state['jobs'])==18 and state['holdout_consumed'] is False
    for job in state['jobs']:
        directory=Path(job['output'])
        policy=json.loads((directory/'policy.json').read_text())
        assert set(policy)=={'planner','parameters','status'}
        manifest=json.loads((directory/'manifest.json').read_text())
        assert manifest['duration']==90. and manifest['official_robot_unchanged']
        assert manifest['sensor_stress_injected'] is False
    with pytest.raises(FileExistsError):
        screen.prepare(output,matrix,campaign)
