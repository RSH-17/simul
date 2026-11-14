import numpy as np
import pytest

from osp_mission.frontier import reachable_gain
from osp_mission.frontier_support import supported_unknown,exploration_gain


def test_small_scan_gaps_do_not_create_exploration_reward_or_change_map():
    data=np.zeros((80,120),dtype=np.int8)
    data[10:70:3,10:110:3]=-1
    original=data.copy();reachable=data==0
    assert reachable_gain(data,.025,reachable).max()>.5
    assert not exploration_gain(data,.025,reachable).any()
    assert np.array_equal(data,original)


@pytest.mark.parametrize('resolution',[.025,.05])
def test_open_unknown_region_retains_original_gain(resolution):
    data=np.zeros((int(5/resolution),int(8/resolution)),dtype=np.int8)
    data[:,int(4/resolution):]=-1
    reachable=data==0
    assert np.allclose(exploration_gain(data,resolution,reachable),reachable_gain(data,resolution,reachable))


def test_narrow_unknown_entrance_to_larger_region_is_retained():
    data=np.zeros((100,100),dtype=np.int8)
    data[30:70,60:90]=-1;data[48:52,20:60]=-1
    selected=supported_unknown(data,.025)
    assert selected[49,21] and selected[50,80]


def test_long_thin_unknown_stripe_is_not_a_new_room():
    data=np.zeros((60,400),dtype=np.int8);data[30:32,10:390]=-1
    assert (data<0).sum()*.025**2>.13
    assert not supported_unknown(data,.025).any()


def test_wall_still_blocks_unknown_reward():
    data=np.zeros((100,100),dtype=np.int8);data[:,50]=100;data[:,51:]=-1
    reachable=data==0
    assert not exploration_gain(data,.025,reachable).any()


def test_crowded_start_uses_original_gain_function():
    from osp_mission import start_waypoints
    assert start_waypoints.reachable_gain is reachable_gain
