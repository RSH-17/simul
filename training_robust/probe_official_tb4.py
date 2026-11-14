"""Read-only inventory of an unmodified official TurtleBot4 simulator."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import time
import xml.etree.ElementTree as ET


def measure(domain, seconds=12.):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from irobot_create_msgs.msg import HazardDetectionVector
    from ament_index_python.packages import get_package_share_directory
    import xacro
    rclpy.init(domain_id=domain)
    node = Node('official_tb4_read_only_probe')
    stamps = defaultdict(list); received = {}; last = {}; description = []
    def receive(key, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if not stamps[key] or stamp > stamps[key][-1]:
            stamps[key].append(stamp)
        received[key] = time.monotonic(); last[key] = message
    for key, topic, kind in [('odom','odom',Odometry),('scan','scan',LaserScan),
            ('rgb','oakd/rgb/preview/image_raw',Image),('info','oakd/rgb/preview/camera_info',CameraInfo),
            ('depth','oakd/rgb/preview/depth',Image),('imu','imu',Imu),('hazards','hazard_detection',HazardDetectionVector)]:
        node.create_subscription(kind, topic, lambda message, k=key: receive(k,message), qos_profile_sensor_data)
    node.create_subscription(String, 'robot_description', lambda message: description.append(message.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        streams = {}
        for key in ['odom','scan','rgb','info','depth','imu','hazards']:
            deltas = [b-a for a,b in zip(stamps[key],stamps[key][1:]) if b>a]
            streams[key] = dict(samples=len(stamps[key]), acquisition_rate_hz=1/statistics.median(deltas) if deltas else None,
                receipt_age_wall_s=time.monotonic()-received[key] if key in received else None)
        if 'scan' in last:
            scan = last['scan']; values = list(scan.ranges); width = len(values)//8
            streams['scan'].update(beams=len(values),angle_min=scan.angle_min,angle_max=scan.angle_max,
                range_min=scan.range_min,range_max=scan.range_max,
                finite_sector_fractions=[sum(math.isfinite(v) for v in values[i*width:(i+1)*width])/width for i in range(8)])
        for key in ['rgb','depth','info']:
            if key in last:
                m = last[key]; streams[key].update(width=m.width,height=m.height,frame=m.header.frame_id)
        if 'info' in last:streams['info']['K']=list(last['info'].k)
        if 'odom' in last:
            m=last['odom'];streams['odom'].update(frame=m.header.frame_id,child=m.child_frame_id,
                position=[m.pose.pose.position.x,m.pose.pose.position.y])
        source=Path(get_package_share_directory('turtlebot4_description'))/'urdf/standard/turtlebot4.urdf.xacro'
        expected=xacro.process_file(str(source),mappings={'gazebo':'ignition','namespace':''}).toxml()
        canonical=lambda xml:ET.canonicalize(xml_data=xml,strip_text=True,with_comments=False)
        actual=canonical(description[-1]) if description else ''
        expected=canonical(expected)
        return dict(domain=domain,commands_published=False,streams=streams,
            official_description_exact_match=actual==expected,
            actual_description_sha256=hashlib.sha256(actual.encode()).hexdigest() if actual else None,
            expected_description_sha256=hashlib.sha256(expected.encode()).hexdigest(),
            velocity_types=dict(node.get_topic_names_and_types()).get('/cmd_vel',[]),
            velocity_consumers=[e.node_name for e in node.get_subscriptions_info_by_topic('cmd_vel')],
            actuator_publishers=[e.node_name for e in node.get_publishers_info_by_topic('diffdrive_controller/cmd_vel')],
            scope='Official simulator input inventory only; no task success or hardware equivalence claim')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(measure(a.domain),indent=2)+'\n')
