import copy

import pytest

from osp_mission.sim_drive_contract import errors


def valid():
    return dict(velocity_consumer_nodes=['motion_control','gazebo_evaluator'],
        drive_publisher_nodes=['motion_control'],drive_consumer_nodes=['diffdrive_controller','cmd_vel_bridge'],
        drive_heartbeat=dict(count=100,age_s=.025))


def test_complete_observed_actuator_path_is_ready():
    assert not errors(valid())


@pytest.mark.parametrize('key,value',[
    ('velocity_consumer_nodes',['gazebo_evaluator']),('drive_publisher_nodes',[]),
    ('drive_consumer_nodes',['cmd_vel_bridge']),('drive_heartbeat',{'count':0,'age_s':None}),
    ('drive_heartbeat',{'count':100,'age_s':3.})])
def test_observers_or_stale_command_stream_cannot_fake_actuator_readiness(key,value):
    observed=copy.deepcopy(valid());observed[key]=value
    assert errors(observed)
