import copy

import pytest

from official_rotation_probe import EXPECTED, configuration_gate


@pytest.mark.parametrize('value,passed', [(True,True),(False,False),(None,False)])
def test_effective_rotation_flag_must_be_observed(value,passed):
    report = dict(parameters=copy.deepcopy(EXPECTED),map=dict(resolution=.05))
    report['parameters']['mission'].update(policy_file='/policy.json',duration=90.)
    key = 'check_min_dist_and_heading_precisely'
    if value is None:
        report['parameters']['slam_toolbox'].pop(key)
    else:
        report['parameters']['slam_toolbox'][key] = value
    assert configuration_gate(report,'/policy.json')['passed'] is passed
