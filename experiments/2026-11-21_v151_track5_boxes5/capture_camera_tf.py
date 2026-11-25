"""Read static camera geometry from simulation only; publish no commands."""
import json
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer,TransformListener

assert os.environ.get('ROS_DOMAIN_ID')=='174' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init();node=Node('read_only_sim_camera_geometry');buffer=Buffer();listener=TransformListener(buffer,node)
infos=[];node.create_subscription(CameraInfo,'/oakd/rgb/preview/camera_info',infos.append,qos_profile_sensor_data)
deadline=time.monotonic()+8;result=None
try:
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if not infos:continue
        info=infos[-1]
        try:tf=buffer.lookup_transform('base_link',info.header.frame_id,Time()).transform
        except Exception:continue
        result=dict(frame=info.header.frame_id,width=info.width,height=info.height,k=list(info.k),
            translation=[tf.translation.x,tf.translation.y,tf.translation.z],
            quaternion=[tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w],
            scope='static simulator camera relative to base_link; read-only, no commands')
        break
    assert result is not None,'No exact camera frame transform received'
    Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:node.destroy_node();rclpy.shutdown()
