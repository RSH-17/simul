import pytest

from osp_mission.exposure_queue import ExposureQueue


@pytest.mark.parametrize('delay_ms,processed', [(0,True),(13,True),(50,True),(200,True),(300,False)])
def test_delayed_transform_is_bounded_and_original_exposure_is_kept(delay_ms,processed):
    q=ExposureQueue();exposure=1_000_000_000;payload=object()
    q.push(exposure,payload,exposure)
    assert q.ready(exposure,lambda *args:False)==[]
    result=q.ready(exposure+delay_ms*1_000_000,lambda stamp,p:stamp==exposure and p is payload)
    assert (result==[payload]) is processed
    assert q.ready(exposure+delay_ms*1_000_000,lambda *args:True)==[]
    assert q.report()['expired']==int(not processed)


def test_missing_transforms_cannot_grow_queue_or_reuse_duplicate_exposures():
    q=ExposureQueue(capacity=3)
    for stamp in range(1,21):
        q.push(stamp,str(stamp),stamp)
        q.ready(stamp,lambda *args:False)
    q.push(20,'duplicate',20);q.push(19,'out_of_order',20)
    assert q.report()['pending']==3 and q.report()['overflow']==17
    assert q.report()['duplicate_or_out_of_order']==2
    assert q.ready(21,lambda *args:True)==['18','19','20']


def test_expired_head_does_not_block_newer_frame_and_clock_reset_discards_old_session():
    q=ExposureQueue();q.push(1_000_000_000,'old',1_000_000_000)
    q.push(1_200_000_000,'new',1_200_000_000)
    assert q.ready(1_300_000_000,lambda stamp,p:p=='new')==['new']
    q.push(1_400_000_000,'before reset',1_400_000_000)
    q.push(10,'after reset',10)
    assert q.ready(11,lambda *args:True)==['after reset']
    assert q.report()['clock_resets']==1
    q.push(2_000_000_000,'invalid future',12)
    assert q.report()['future']==1 and q.report()['pending']==0
    q.push(0,'unknown exposure must not request latest TF',12)
    assert q.report()['invalid_timestamp']==1 and q.report()['pending']==0


@pytest.mark.parametrize('use_depth',[False,True])
def test_detector_retries_delayed_tf_with_original_rgb_calibration_and_range(use_depth):
    # Real tf2 Buffer interpolation and production detector, no robot/TF publisher.
    from types import SimpleNamespace as S
    import cv2
    import numpy as np
    import rclpy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo,LaserScan
    from geometry_msgs.msg import TransformStamped
    from cv_bridge import CvBridge
    from osp_mission.detector import Detector

    rclpy.init();node=Detector();bridge=CvBridge();clock=[1_000_000_000]
    node.get_clock=lambda:S(now=lambda:Time(nanoseconds=clock[0]))
    heartbeats=[];node.processed_camera=S(publish=heartbeats.append)

    def transform(stamp,x):
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='camera'
        t.header.stamp=Time(nanoseconds=stamp).to_msg();t.transform.translation.x=x
        t.transform.rotation.x=-2**-.5;t.transform.rotation.w=2**-.5
        node.tf.set_transform(t,'test_sensor_delay')

    try:
        laser=TransformStamped();laser.header.frame_id='map';laser.child_frame_id='laser'
        laser.transform.rotation.w=1.;node.tf.set_transform_static(laser,'test_sensor_delay')
        info=CameraInfo();info.width=640;info.height=480
        info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.];info.d=[0.]*5
        gray=np.full((480,640),255,dtype=np.uint8)
        gray[190:290,270:370]=cv2.aruco.drawMarker(node.dictionary,42,100)
        for i in range(3):
            stamp=1_000_000_000+i*200_000_000;clock[0]=stamp
            transform(stamp-13_000_000,i*.05)
            rgb=bridge.cv2_to_imgmsg(gray,'mono8');rgb.header.frame_id='camera'
            rgb.header.stamp=Time(nanoseconds=stamp).to_msg()
            depth=bridge.cv2_to_imgmsg(np.full((480,640),2.,np.float32),'32FC1');depth.header=rgb.header
            angles=np.linspace(1.45,1.69,121)
            scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=rgb.header.stamp
            scan.angle_min=float(angles[0]);scan.angle_increment=float(angles[1]-angles[0])
            scan.range_min=.16;scan.range_max=12.;scan.ranges=(2/np.sin(angles)).tolist()
            node.on_info(info)
            if use_depth:node.on_depth(depth)
            else:node.on_scan(scan)
            node.queue_rgb(rgb)
            assert len(heartbeats)==i  # Future TF is unavailable; no false inspection.
            newer=bridge.cv2_to_imgmsg(np.full((480,640),4.,np.float32),'32FC1');newer.header=rgb.header
            invalid_info=CameraInfo();invalid_info.width=320
            import copy
            newer_scan=copy.deepcopy(scan);newer_scan.ranges=(4/np.sin(angles)).tolist()
            node.on_info(invalid_info)
            if use_depth:node.on_depth(newer)
            else:node.on_scan(newer_scan)
            clock[0]=stamp+13_000_000;transform(clock[0],i*.05+.026)
            node.drain_rgb()
            assert node.info is invalid_info
            assert node.depth is newer if use_depth else node.scan is newer_scan
            assert heartbeats[-1].header.stamp==rgb.header.stamp
            observation=node.observations[42][-1]
            assert observation[0]==stamp
            assert observation[1][0]==pytest.approx(i*.05+.011,abs=.001)
            assert observation[1][1]==pytest.approx(2.,abs=.001)
            node.queue_rgb(rgb);node.drain_rgb()
            assert len(heartbeats)==i+1
        assert len(node.observations[42])==3
        assert node.diagnostics['errors']==0
        assert node.exposures.report()['duplicate_or_out_of_order']==3
    finally:
        node.destroy_node();rclpy.shutdown()
