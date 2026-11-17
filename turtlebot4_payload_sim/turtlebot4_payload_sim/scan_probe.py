"""Run one isolated Gazebo variant and record observed ROS LaserScan samples."""
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

from turtlebot4_payload_sim.model import VARIANTS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', choices=VARIANTS, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--timeout-wall', type=float, default=90.)
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--payload-config', type=Path)
    args = parser.parse_args()
    if args.samples <= 0 or args.timeout_wall <= 0 or not 0 <= args.domain <= 230:
        parser.error('samples and timeout-wall must be positive; domain must be 0..230')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                       GZ_PARTITION=f'payload_{args.variant}_{os.getpid()}')
    log = (args.output_dir/'launch.log').open('w')
    command = ['ros2', 'launch', 'turtlebot4_payload_sim', 'payload_sim.launch.py',
               f'variant:={args.variant}',
               f'navigation:={str(args.navigation).lower()}']
    if args.payload_config:
        command.append(f'payload_config:={args.payload_config}')
    process = subprocess.Popen(
        command,
        env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = rclpy.create_node(f'payload_scan_probe_{args.variant}')
    rows = []
    started = time.monotonic()

    def on_scan(message):
        valid = [(i, float(distance)) for i, distance in enumerate(message.ranges)
                 if math.isfinite(distance) and message.range_min < distance < message.range_max]
        near = [(i, distance) for i, distance in valid if distance < 0.4]
        closest = min(valid, key=lambda item: item[1]) if valid else None
        rows.append({
            'wall_elapsed_s': round(time.monotonic()-started, 3),
            'ros_stamp_s': message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            'scan_frame': message.header.frame_id,
            'range_min_m': message.range_min,
            'valid_returns': len(valid),
            'within_40cm': len(near),
            'minimum_range_m': closest[1] if closest else '',
            'minimum_angle_rad': message.angle_min+closest[0]*message.angle_increment if closest else '',
            'minimum_beam_index': closest[0] if closest else '',
        })

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    try:
        deadline = started+args.timeout_wall
        while time.monotonic() < deadline and len(rows) < args.samples:
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
    with (args.output_dir/'scan.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else
                                ['wall_elapsed_s','ros_stamp_s','scan_frame','range_min_m',
                                 'valid_returns','within_40cm','minimum_range_m',
                                 'minimum_angle_rad','minimum_beam_index'])
        writer.writeheader()
        writer.writerows(rows)
    summary = {'variant': args.variant, 'samples': len(rows),
               'payload_config': str(args.payload_config) if args.payload_config else 'default',
               'minimum_range_m': min((r['minimum_range_m'] for r in rows
                                       if r['minimum_range_m'] != ''), default=None),
               'max_within_40cm': max((r['within_40cm'] for r in rows), default=None),
               'wall_seconds': round(time.monotonic()-started, 3),
               'navigation': args.navigation,
               'scope': 'Observed /scan only; no navigation or physical-robot conclusion'}
    (args.output_dir/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
    if not rows:
        raise SystemExit('No LaserScan samples; inspect launch.log')


if __name__ == '__main__':
    main()
