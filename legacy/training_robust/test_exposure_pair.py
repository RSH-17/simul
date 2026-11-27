"""ROS integration checks; build the two frozen workspaces before running."""
from pathlib import Path
import sys
from types import SimpleNamespace as S

import pytest

from observe_exposure_pair import detector_class,owner,owned_publishers


def test_paired_observer_has_no_policy_publishers_and_recovers_delayed_tf():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo
    from cv_bridge import CvBridge
    from tf2_ros import Buffer,TransformListener

    root=Path(__file__).resolve().parents[1]
    baseline=root/'training_robust/native90/official_tb4_v11/runtime'
    candidate=root/'training_robust/native90/official_tb4_v13/runtime'
    if not (candidate/'install/setup.bash').exists():pytest.skip('Build the frozen exposure candidate first')
    sys.path.insert(0,str(candidate/'src/osp_mission'))
    original,_=detector_class(baseline,'test_pair_original')
    buffered,_=detector_class(candidate,'test_pair_buffered')
    rclpy.init(domain_id=224)
    node=rclpy.create_node('test_exposure_pair',enable_rosout=False,start_parameter_services=False)
    buffer=Buffer(node=node);listener=TransformListener(buffer,node);events=[];now=[1_000_000_000]
    try:
        a=owner(original,node,buffer,lambda kind,data:events.append(('original',kind,data)))
        b=owner(buffered,node,buffer,lambda kind,data:events.append(('buffered',kind,data)))
        assert type(a) is S and type(b) is S  # No Detector node/constructor.
        assert owned_publishers(node)==['/parameter_events']
        info=CameraInfo();info.width=640;info.height=480
        info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.];info.d=[0.]*5
        gray=np.full((480,640),255,np.uint8)
        gray[190:290,270:370]=cv2.aruco.drawMarker(a.dictionary,42,100)
        bridge=CvBridge();rgb=bridge.cv2_to_imgmsg(gray,'mono8')
        rgb.header.frame_id='camera';rgb.header.stamp=Time(nanoseconds=now[0]).to_msg()
        depth=bridge.cv2_to_imgmsg(np.full((480,640),2.,np.float32),'32FC1');depth.header=rgb.header
        for n in (a,b):
            n.info=info;n.depth=depth;n.get_clock=lambda:S(now=lambda:Time(nanoseconds=now[0]))
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='camera'
        transform.transform.rotation.x=-2**-.5;transform.transform.rotation.w=2**-.5
        transform.header.stamp=Time(nanoseconds=now[0]-13_000_000).to_msg()
        buffer.set_transform(transform,'read_only_test')
        a.on_rgb(rgb);b.queue_rgb(rgb)
        assert a.diagnostics['errors']==1 and b.diagnostics['errors']==0
        assert not a.observations and not b.observations
        now[0]+=13_000_000;transform.header.stamp=Time(nanoseconds=now[0]).to_msg()
        buffer.set_transform(transform,'read_only_test');b.drain_rgb()
        assert len(b.observations[42])==1 and not a.observations
        assert b.observations[42][0][0]==1_000_000_000
        assert b.observations[42][0][1][1]==pytest.approx(2.)
        assert any(role=='buffered' and kind=='observation' for role,kind,data in events)
        assert owned_publishers(node)==['/parameter_events']
        assert not hasattr(a,'create_publisher') and not hasattr(b,'create_publisher')
    finally:
        node.destroy_node();rclpy.shutdown()
