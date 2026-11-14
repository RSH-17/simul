import copy
import pytest
from official_no_return_probe import EXPECTED
from official_fine_no_return_probe import configuration_gate


@pytest.mark.parametrize('parameter,grid,passed',[(.025,.025,True),(.05,.025,False),(.025,.05,False)])
def test_both_effective_parameter_and_published_grid_must_be_fine(parameter,grid,passed):
    report=dict(parameters=copy.deepcopy(EXPECTED),map=dict(resolution=grid))
    report['parameters']['mission'].update(policy_file='/p',duration=90.)
    report['parameters']['slam_toolbox']['resolution']=parameter
    assert configuration_gate(report,'/p')['passed'] is passed
