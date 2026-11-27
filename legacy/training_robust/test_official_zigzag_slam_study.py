import copy
import pytest

from run_official_zigzag_slam_study import completed_pair


def pair():
    return dict(jobs=[dict(number=i,role=role,status='done',outcome=dict(infrastructure_failure=False),
        scenario=dict(sha256='same-recorded-scene',arena=dict(shape='zigzag')))
        for i,role in [(4,'baseline'),(5,'candidate')]])


@pytest.mark.parametrize('field,value',[('status','running'),('infrastructure_failure',True)])
def test_live_or_invalid_recordings_cannot_become_completed_comparisons(field,value):
    state=pair()
    (state['jobs'][1]['outcome'] if field=='infrastructure_failure' else state['jobs'][1])[field]=value
    with pytest.raises(ValueError,match='completed verified'):completed_pair(state)


def test_scene_mismatch_is_rejected_and_valid_pair_is_unmodified():
    state=pair();before=copy.deepcopy(state)
    assert completed_pair(state)==before['jobs'] and state==before
    state['jobs'][1]['scenario']['sha256']='different'
    with pytest.raises(ValueError,match='identical'):completed_pair(state)
