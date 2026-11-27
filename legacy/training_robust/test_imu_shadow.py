import numpy as np
import pytest

from audit_imu_shadow import compare, ordered
from probe_official_imu_odometry import parameters, child_arguments


def test_same_motion_different_frame_vs_growing_encoder_error():
    t=np.linspace(0,3,151)
    truth=ordered(np.column_stack([t,np.sin(t),1-np.cos(t),t]))
    angle=.7;c,s=np.cos(angle),np.sin(angle);r=np.array([[c,-s],[s,c]])
    exact=truth.copy();exact[:,1:3]=(truth[:,1:3]-[4,-3])@r;exact[:,3]-=angle
    drift=exact.copy();drift[:,3]+=.2*t
    result=compare(truth,{'ekf':exact,'wheel':drift})
    assert result['estimators']['ekf']['position_error_m']['maximum']<1e-10
    assert result['estimators']['ekf']['heading_error_deg']['maximum']<1e-10
    assert result['estimators']['wheel']['final_heading_error_deg']>33
    assert result['actual_absolute_turn_rad']==pytest.approx(3)


def test_missing_truth_is_not_bridged():
    t=np.linspace(0,3,151);a=ordered(np.column_stack([t,t,t*0,t*0]))
    truth=a[(t<.5)|(t>2.5)]
    result=compare(truth,{'wheel':a,'ekf':a})
    assert result['estimators']['ekf']['position_error_m']['samples']<55


def test_shadow_cannot_control_robot_or_use_global_pose():
    p=parameters()
    assert p['publish_tf'] is False and p['use_control'] is False
    assert [i for i,v in enumerate(p['odom0_config']) if v]==[6,7]
    assert [i for i,v in enumerate(p['imu0_config']) if v]==[11]
    assert p['odom0']=='odom' and p['imu0']=='imu'
    args=child_arguments('/tmp/test.yaml')
    assert '/tf:=diagnostics/imu_shadow_tf' in args
    assert '/tf_static:=diagnostics/imu_shadow_tf_static' in args
    assert 'odometry/filtered:=diagnostics/official_imu_odometry' in args
