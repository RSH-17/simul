"""Preserve Gazebo's positive-infinite no-return rays for Karto rasterization.

Only the SLAM copy is normalized. Original scan, collision monitoring, detector,
and platform checks remain on the unmodified sensor stream. Karto must use a
range threshold of 11.9 m: the substituted 11.95 m values are clearing rays,
never obstacle endpoints or scan-matching features. This is not a hardware
dropout repair; physical sensor no-return semantics have not been validated.
"""
import copy
import math

import numpy as np


def mapping_ranges(ranges, range_min, range_max):
    if not math.isfinite(range_min) or not 0 < range_min < 11.9:
        raise ValueError('Invalid scan minimum range')
    if not math.isfinite(range_max) or not math.isclose(range_max, 12., abs_tol=1e-6):
        raise ValueError('Expected official simulator 12 m sensor')
    values = np.asarray(ranges, dtype=np.float32).copy()
    values[np.isposinf(values)] = 11.95
    return values.tolist()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('sim_mapping_scan')
    if not node.has_parameter('use_sim_time'):
        node.declare_parameter('use_sim_time', False)
    if not node.get_parameter('use_sim_time').value:
        raise RuntimeError('This sensor adapter is restricted to simulation')
    node.declare_parameter('input_topic', 'scan')
    publisher = node.create_publisher(LaserScan, 'scan_mapping', qos_profile_sensor_data)

    def receive(message):
        try:
            ranges = mapping_ranges(message.ranges, message.range_min, message.range_max)
        except ValueError as error:
            node.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        output = copy.deepcopy(message)
        output.ranges = ranges
        publisher.publish(output)

    node.create_subscription(LaserScan, node.get_parameter('input_topic').value,
                             receive, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
