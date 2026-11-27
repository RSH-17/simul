import copy
import pytest
from official_no_return_probe import EXPECTED
from official_training_input_probe import configuration_gate


@pytest.mark.parametrize('adapter,passed',[(True,True),(False,False),(None,False)])
def test_full_task_configuration_requires_observed_scan_adapter(adapter,passed):
    report=dict(parameters=copy.deepcopy(EXPECTED),map=dict(resolution=.025),scan_adapter=dict(passed=adapter))
    report['parameters']['mission'].update(policy_file='/p',duration=600.)
    report['parameters']['slam_toolbox']['resolution']=.025
    assert configuration_gate(report,'/p')['passed'] is passed
    report['parameters']['mission']['duration']=90.
    assert not configuration_gate(report,'/p')['passed']
