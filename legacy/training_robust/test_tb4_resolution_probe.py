"""Native acceptance requires the actual published map and parameter to agree."""
import copy

import pytest

from tb4_platform_probe_v2 import EXPECTED, configuration_gate


@pytest.mark.parametrize('expected', [.05, .025])
def test_correct_resolution_in_both_parameter_and_message(expected):
    report = dict(parameters=copy.deepcopy(EXPECTED), map=dict(resolution=expected))
    report['parameters']['slam_toolbox']['resolution'] = expected
    report['parameters']['mission']['policy_file'] = '/policy.json'
    assert configuration_gate(report, '/policy.json', expected_resolution=expected)['passed']


@pytest.mark.parametrize('wrong', ['parameter', 'message', 'absent'])
def test_map_resolution_mismatch_cannot_qualify(wrong):
    report = dict(parameters=copy.deepcopy(EXPECTED), map=dict(resolution=.025))
    report['parameters']['mission']['policy_file'] = '/policy.json'
    if wrong == 'parameter':
        report['parameters']['slam_toolbox']['resolution'] = .05
    elif wrong == 'message':
        report['map']['resolution'] = .05
    else:
        report.pop('map')
    assert not configuration_gate(report, '/policy.json')['passed']
