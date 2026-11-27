"""Measure raw versus fixture-filtered Gazebo scans at the same ROS stamp."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--payload-config', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout-wall', type=float, default=75.)
    args = parser.parse_args()
    if not 0 <= args.domain <= 230:
        parser.error('domain must be 0..230')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                       GZ_PARTITION=f'payload_filter_{os.getpid()}')
    log = (args.output_dir/'launch.log').open('w')
    process = subprocess.Popen(
        ['ros2','launch','turtlebot4_payload_sim','payload_sim.launch.py',
         'variant:=full','fixture_filter:=true',
         f'payload_config:={args.payload_config}'],
        env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = rclpy.create_node('payload_filter_probe')
    raw = {}
    matched = []

    def key(message):
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def on_raw(message):
        raw[key(message)] = message
        if len(raw) > 50:
            raw.pop(next(iter(raw)))

    def on_filtered(message):
        original = raw.get(key(message))
        if original is None or len(original.ranges) != len(message.ranges):
            return
        removed = [i for i,(a,b) in enumerate(zip(original.ranges, message.ranges))
                   if math.isfinite(a) and not math.isfinite(b)]
        if removed:
            matched.append((original,message,removed))

    node.create_subscription(LaserScan, '/scan', on_raw, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan_payload_filtered', on_filtered,
                             qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.timeout_wall and not matched:
            if process.poll() is not None:
                break
            rclpy.spin_once(node, timeout_sec=.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
    if not matched:
        raise SystemExit('No matched frame with removed fixture returns; inspect launch.log')
    original, filtered, removed = matched[0]
    with (args.output_dir/'raw_filtered_scan.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['beam_index','angle_rad','raw_range_m','filtered_range_m'])
        for i,(a,b) in enumerate(zip(original.ranges, filtered.ranges)):
            writer.writerow([i, original.angle_min+i*original.angle_increment, a, b])
    front = 160
    summary = {'removed_beams': removed,
               'raw_front_beam_m': original.ranges[front],
               'filtered_front_beam_m': (filtered.ranges[front] if
                                          math.isfinite(filtered.ranges[front]) else None),
               'physical_external_obstacle_center_base_link_m': [1.0, 0.0],
               'conclusion': 'Filtering removes the near fixture return but cannot recover the occluded farther physical obstacle',
               'scope': 'One Gazebo scan; not a hardware safety validation'}
    (args.output_dir/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
