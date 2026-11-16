"""Remove returns wholly inside the robot body before mapping/navigation.

The 0.16 m limit is inside the 0.189 m TurtleBot4 Standard footprint.
No angular sector or point outside that inner disc is suppressed.
"""
import copy
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


INNER_RADIUS_M = .16


def filter_ranges(scan, sensor_x, sensor_y, sensor_yaw, radius=INNER_RADIUS_M):
    """Replace only finite returns strictly inside the body with no return."""
    if not (0. < radius <= INNER_RADIUS_M):
        raise ValueError('Self-filter radius must stay inside the body')
    if not all(math.isfinite(v) for v in (sensor_x, sensor_y, sensor_yaw,
                                         scan.angle_min, scan.angle_increment)):
        raise ValueError('Invalid scan transform or angular metadata')
    ranges = list(scan.ranges)
    removed = 0
    for index, distance in enumerate(ranges):
        if not (math.isfinite(distance) and scan.range_min < distance < scan.range_max):
            continue
        angle = sensor_yaw + scan.angle_min + index * scan.angle_increment
        x = sensor_x + distance * math.cos(angle)
        y = sensor_y + distance * math.sin(angle)
        if x*x + y*y < radius*radius:
            ranges[index] = math.inf
            removed += 1
    return ranges, removed


class SelfScanFilter(Node):
    def __init__(self):
        super().__init__('self_scan_filter')
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_self_filtered')
        self.declare_parameter('base_frame', 'base_link')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.total_removed = 0
        self.total_scans = 0

    def on_scan(self, scan):
        try:
            transform = self.tf.lookup_transform(
                self.get_parameter('base_frame').value, scan.header.frame_id,
                Time.from_msg(scan.header.stamp), timeout=Duration(seconds=.05)).transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1-2*(q.y*q.y + q.z*q.z))
            ranges, removed = filter_ranges(
                scan, transform.translation.x, transform.translation.y, yaw)
            result = copy.deepcopy(scan)
            result.ranges = ranges
            self.total_removed += removed
            self.total_scans += 1
            if self.total_scans % 30 == 0:
                self.get_logger().info(
                    f'Self returns removed: {self.total_removed} / {self.total_scans} scans')
        except Exception as exc:
            # Preserve raw observations when geometry cannot be established.
            self.get_logger().warning(f'Self scan filter unavailable: {exc}',
                                      throttle_duration_sec=5.)
            result = scan
        self.output.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = SelfScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
