import copy

import pytest

from official_no_return_probe import EXPECTED, configuration_gate, paired_scan_gate


@pytest.mark.parametrize('key,value', [('scan_topic','scan'),('max_laser_range',12.),('max_laser_range',None)])
def test_wrong_slam_routing_or_threshold_is_rejected(key,value):
    report=dict(parameters=copy.deepcopy(EXPECTED),map=dict(resolution=.05))
    report['parameters']['mission'].update(policy_file='/p',duration=90.)
    assert configuration_gate(report,'/p')['passed']
    report['parameters']['slam_toolbox'][key]=value
    assert not configuration_gate(report,'/p')['passed']


def test_raw_invalid_returns_and_metadata_cannot_be_rewritten():
    raw=dict(frame='laser',stamp=(1,0),range_min=.164,range_max=12.,angle_min=-3.14,
        angle_max=3.14,angle_increment=.01,time_increment=0.,scan_time=0.,intensities=[],
        ranges=[float('inf'),float('-inf'),float('nan'),0.,.5])
    mapped=copy.deepcopy(raw);mapped['ranges'][0]=11.95
    assert paired_scan_gate(raw,mapped)['passed']
    for index in range(1,5):
        broken=copy.deepcopy(mapped);broken['ranges'][index]=11.95
        assert not paired_scan_gate(raw,broken)['passed']
    mapped['stamp']=(2,0)
    assert not paired_scan_gate(raw,mapped)['passed']
