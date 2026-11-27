import copy
import numpy as np
import pytest
from osp_mission.validated_graph_landmarks import finalize
from osp_mission.pose_increment_gyro import integrate
from osp_mission.pose_gyro_stream import PoseGyroStream
from osp_mission.pose_gyro_shadow import rotation


def graph_inputs(shift):
    graph=np.array([[i,t,0,0,0,shift,0,0] for i,t in enumerate([1.,1.2,1.4,1.6])])
    rows=[dict(id=7,stamp=t,odom_point=[1.,0.,.45],min_edge_pixels=24.) for t in [1.,1.2,1.4]]
    return rows,graph,{7:(1.,0.)}


def test_consistent_large_bad_graph_cannot_overwrite_online_coordinate_or_claim_verification():
    rows,graph,old=graph_inputs(1.5);before=copy.deepcopy(rows)
    final,report=finalize(rows,graph,old)
    assert final==old and report['corrected_ids']==[]
    assert report['requires_reobservation']==[7] and not report['coordinates_verified']
    assert report['rows'][0]['proposed_point']==[2.5,0.]
    assert rows==before


@pytest.mark.parametrize('shift',[-.25,0.,.10,.25])
def test_bounded_consistent_graph_correction_remains_enabled(shift):
    rows,graph,old=graph_inputs(shift);final,report=finalize(rows,graph,old)
    assert final[7]==pytest.approx((1.+shift,0.)) and report['coordinates_verified']


def test_insufficient_scan_support_never_becomes_coordinate_verification():
    rows,graph,old=graph_inputs(.1);final,report=finalize(rows[:1],graph,old)
    assert final==old and not report['coordinates_verified'] and report['requires_reobservation']==[7]


def test_stream_matches_batch_with_delayed_wheel_delivery_and_yaw_wrap():
    t=np.arange(0,8,.01);w=1.1;v=.2
    wheel=np.column_stack([t,v/w*np.sin(w*t),v/w*(1-np.cos(w*t)),(w*t+np.pi)%(2*np.pi)-np.pi])
    gyro=np.column_stack([t,np.ones(len(t))]);expected=integrate(wheel,gyro)
    stream=PoseGyroStream();rows=[]
    for i in range(len(t)):
        rows+=stream.imu(*gyro[i])
        if i>=2:rows+=stream.wheel(wheel[i-2])
    rows+=stream.wheel(wheel[-2]);rows+=stream.wheel(wheel[-1])
    assert np.allclose(rows,expected,atol=1e-11)
    assert len(stream.gyro)<8


@pytest.mark.parametrize('fault',['gyro_gap','wheel_reset','conflict','nonfinite'])
def test_stream_faults_are_latched_without_fabricated_reset(fault):
    stream=PoseGyroStream();stream.imu(0.,0.);stream.imu(.01,0.);stream.wheel([0.,0.,0.,0.])
    with pytest.raises(ValueError):
        if fault=='gyro_gap':stream.imu(.2,0.)
        elif fault=='wheel_reset':stream.wheel([-.1,0.,0.,0.])
        elif fault=='conflict':stream.wheel([0.,.1,0.,0.])
        else:stream.imu(.02,float('nan'))
    assert stream.fault
    with pytest.raises(ValueError):stream.imu(.03,0.)


def test_mount_rotation_transforms_full_gyro_vector_and_rejects_invalid_quaternion():
    r=rotation([np.sin(np.pi/4),0.,0.,np.cos(np.pi/4)])
    assert r@np.array([0.,1.,0.])==pytest.approx([0.,0.,1.])
    with pytest.raises(ValueError):rotation([0,0,0,0])


def test_bias_is_not_magically_removed_from_sensor_only_integration():
    t=np.arange(0,3,.01);wheel=np.column_stack([t,.2*t,0*t,0*t])
    a=integrate(wheel,np.column_stack([t,0*t]));b=integrate(wheel,np.column_stack([t,0*t+.02]))
    assert b[-1,3]-a[-1,3]==pytest.approx(.02*t[-1])
