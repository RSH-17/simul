"""Experimental fixture-only LaserScan filter; raw /scan stays available.

Filtering cannot reveal an external obstacle hidden behind a physical fixture.
Never use this output on a real robot before same-ray obstacle safety testing.
"""
import argparse
import copy
import math
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import yaml


def rotate(q, x, y, z):
    # Quaternion vector rotation; handles pitch/roll from the robot physics.
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx, ty, tz = (2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x))
    return (x+qw*tx+qy*tz-qz*ty,
            y+qw*ty+qz*tx-qx*tz,
            z+qw*tz+qx*ty-qy*tx)


def segment_distance(p, a, b):
    ab = [b[i]-a[i] for i in range(3)]
    ap = [p[i]-a[i] for i in range(3)]
    length2 = sum(v*v for v in ab)
    t = max(0., min(1., sum(ap[i]*ab[i] for i in range(3))/length2))
    return math.sqrt(sum((p[i]-a[i]-t*ab[i])**2 for i in range(3)))


def inside_fixture(p, config, margin=.015):
    adapter = config['adapter']
    center, size = adapter['center_m'], adapter['size_m']
    yaw = adapter['rpy_rad'][2]
    dx, dy, dz = (p[i]-center[i] for i in range(3))
    local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
    if all(abs(v) <= size[i]/2+margin for i, v in enumerate((local_x, local_y, dz))):
        return True
    cable = config['cable']
    return any(segment_distance(p, a, b) <= cable['diameter_m']/2+margin
               for a, b in zip(cable['points_m'], cable['points_m'][1:]))


def filter_ranges(scan, transform, config, margin=.015):
    result = list(scan.ranges)
    removed = 0
    for i, distance in enumerate(scan.ranges):
        if not (math.isfinite(distance) and scan.range_min < distance < scan.range_max):
            continue
        angle = scan.angle_min+i*scan.angle_increment
        v = rotate(transform.rotation, distance*math.cos(angle),
                   distance*math.sin(angle), 0.)
        point = [v[0]+transform.translation.x,
                 v[1]+transform.translation.y,
                 v[2]+transform.translation.z]
        if inside_fixture(point, config, margin):
            result[i] = math.inf
            removed += 1
    return result, removed


class FixtureFilter(Node):
    def __init__(self, config):
        super().__init__('payload_fixture_filter',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.config = config
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, '/scan_payload_filtered',
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.total_removed = 0

    def on_scan(self, scan):
        try:
            tr = self.tf.lookup_transform('base_link', scan.header.frame_id,
                                          Time.from_msg(scan.header.stamp),
                                          timeout=Duration(seconds=.05)).transform
            ranges, removed = filter_ranges(scan, tr, self.config)
            out = copy.deepcopy(scan)
            out.ranges = ranges
            self.total_removed += removed
            if removed:
                self.get_logger().info(f'Removed {removed} fixture returns from scan',
                                       throttle_duration_sec=5.)
        except Exception as exc:
            self.get_logger().warning(f'Fixture filter passed raw scan: {exc}',
                                      throttle_duration_sec=5.)
            out = scan
        self.pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = FixtureFilter(yaml.safe_load(args.config.read_text()))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
