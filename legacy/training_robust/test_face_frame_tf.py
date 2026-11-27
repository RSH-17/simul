"""Exercise the candidate against real tf2 composition without ROS publishers."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer


@pytest.mark.parametrize('selection_rotation,arrival_rotation',[(0.,.4),(.7,-.9),(-2.9,2.9)])
def test_face_sweep_matches_map_goal_after_real_tf2_odometry_correction(selection_rotation,arrival_rotation):
    source=Path(__file__).resolve().parents[1]/'policy_candidates/2026-11-15_official_tb4_v14/overlay/osp_mission/face_directed_look.py'
    spec=importlib.util.spec_from_file_location('candidate_face_tf',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    buffer=Buffer()
    def yaw_transform(parent,child,yaw):
        tr=TransformStamped();tr.header.stamp.sec=61;tr.header.frame_id=parent;tr.child_frame_id=child
        tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2)
        buffer.set_transform(tr,'test_sensor_estimate')
    yaw_transform('building_map','odom',-arrival_rotation)
    yaw_transform('odom','base_link',.3+arrival_rotation)
    optical=TransformStamped();optical.header.frame_id='base_link';optical.child_frame_id='rgb_optical'
    optical.transform.rotation.x=-.5;optical.transform.rotation.y=.5
    optical.transform.rotation.z=-.5;optical.transform.rotation.w=.5
    buffer.set_transform_static(optical,'test_sensor_mount')
    owner=NS(tf=buffer,inspection_camera_frame='rgb_optical',
             p=lambda name:'building_map' if name=='map_frame' else None,
             get_logger=lambda:NS(info=lambda *args:None))
    look=module.FaceDirectedLook(owner);look.event=lambda *args,**kwargs:None
    look.select(dict(face_inspection=True,yaw=.2,scan_angle=math.pi/3),(4.,-3.,selection_rotation))
    assert look.consume()==pytest.approx(math.pi/3-.1)
    assert look.pending is None
