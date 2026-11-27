"""Open-loop physical response test; bypasses Nav2 only for payload dynamics."""
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
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', choices=['baseline','mass_only'], required=True)
    parser.add_argument('--payload-config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--timeout-wall', type=float, default=80.)
    parser.add_argument('--profile', choices=['straight', 'turn'], default='straight')
    args = parser.parse_args()
    if not 0 <= args.domain <= 230:
        parser.error('domain must be 0..230')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                       GZ_PARTITION=f'payload_motion_{os.getpid()}')
    log = (args.output_dir/'launch.log').open('w')
    process = subprocess.Popen(
        ['ros2','launch','turtlebot4_payload_sim','payload_sim.launch.py',
         f'variant:={args.variant}',f'payload_config:={args.payload_config}'],
        env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = rclpy.create_node('payload_motion_probe',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    rows = []
    latest_imu = {'roll': None, 'pitch': None, 'horizontal_accel': None}
    first_odom = None
    latest_position = None
    previous = None
    sim_start = None
    settle_start = None
    command_off_sim = None
    hazard_types = set()
    hazard_events = {}
    data = {'max_speed_mps': 0., 'max_odom_accel_mps2': 0.,
            'max_angular_speed_radps': 0., 'max_angular_accel_radps2': 0.,
            'max_abs_roll_rad': 0., 'max_abs_pitch_rad': 0.,
            'max_horizontal_imu_accel_mps2': 0.}

    def on_hazard(message):
        if sim_start is None:
            return
        for detection in message.detections:
            kind = int(detection.type)
            hazard_types.add(kind)
            key = f'{kind}:{detection.header.frame_id}'
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            event = hazard_events.setdefault(key, {'first_sim_s': stamp,
                                                   'last_sim_s': stamp, 'count': 0})
            event['last_sim_s'] = stamp
            event['count'] += 1

    def on_imu(message):
        q = message.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        horizontal = math.hypot(message.linear_acceleration.x,
                                message.linear_acceleration.y)
        latest_imu.update(roll=roll, pitch=pitch, horizontal_accel=horizontal)
        if sim_start is not None:
            data['max_abs_roll_rad'] = max(data['max_abs_roll_rad'], abs(roll))
            data['max_abs_pitch_rad'] = max(data['max_abs_pitch_rad'], abs(pitch))
            data['max_horizontal_imu_accel_mps2'] = max(
                data['max_horizontal_imu_accel_mps2'], horizontal)

    def on_odom(message):
        nonlocal first_odom, previous, latest_position
        p = message.pose.pose.position
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        latest_position = (p.x,p.y)
        if sim_start is None:
            return
        distance = math.hypot(p.x-first_odom[0], p.y-first_odom[1])
        speed = message.twist.twist.linear.x
        angular = message.twist.twist.angular.z
        q = message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        data['max_speed_mps'] = max(data['max_speed_mps'], abs(speed))
        data['max_angular_speed_radps'] = max(data['max_angular_speed_radps'], abs(angular))
        if previous and stamp > previous[0]:
            data['max_odom_accel_mps2'] = max(
                data['max_odom_accel_mps2'], abs(speed-previous[1])/(stamp-previous[0]))
            data['max_angular_accel_radps2'] = max(
                data['max_angular_accel_radps2'], abs(angular-previous[2])/(stamp-previous[0]))
        previous = (stamp,speed,angular)
        rows.append({'sim_stamp_s':stamp, 'wall_stamp_s':time.time(),
                     'x_m':p.x, 'y_m':p.y, 'distance_m':distance, 'speed_mps':speed,
                     'angular_speed_radps':angular,
                     'yaw_rad':yaw,
                     'roll_rad':latest_imu['roll'], 'pitch_rad':latest_imu['pitch'],
                     'horizontal_imu_accel_mps2':latest_imu['horizontal_accel']})

    node.create_subscription(Odometry, '/odom', on_odom, 10)
    node.create_subscription(Imu, '/imu', on_imu, 10)
    node.create_subscription(HazardDetectionVector, '/hazard_detection', on_hazard, 10)
    wall_start = time.monotonic()
    last_pub = 0.
    try:
        while time.monotonic()-wall_start < args.timeout_wall:
            if process.poll() is not None:
                data['reason'] = 'simulator_exited'
                break
            rclpy.spin_once(node, timeout_sec=.04)
            now_sim = node.get_clock().now().nanoseconds/1e9
            if latest_position is None or now_sim <= 0:
                continue
            if settle_start is None:
                settle_start = now_sim
            if now_sim-settle_start < 2.:
                if time.monotonic()-last_pub >= .04:
                    zero = TwistStamped()
                    zero.header.stamp = node.get_clock().now().to_msg()
                    pub.publish(zero)
                    last_pub = time.monotonic()
                continue
            if sim_start is None:
                sim_start = now_sim
                first_odom = latest_position
                previous = None
            elapsed = now_sim-sim_start
            if elapsed >= 2. and command_off_sim is None:
                command_off_sim = now_sim
            if elapsed >= 3.:
                data['reason'] = 'completed'
                break
            if time.monotonic()-last_pub >= .04:
                command = TwistStamped()
                command.header.stamp = node.get_clock().now().to_msg()
                if args.profile == 'straight':
                    command.twist.linear.x = .2 if elapsed < 2. else 0.
                else:
                    command.twist.angular.z = .6 if elapsed < 2. else 0.
                pub.publish(command)
                last_pub = time.monotonic()
        else:
            data['reason'] = 'wall_timeout'
        for _ in range(5):
            zero = TwistStamped()
            zero.header.stamp = node.get_clock().now().to_msg()
            pub.publish(zero)
            rclpy.spin_once(node, timeout_sec=.05)
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
    after_stop = [r for r in rows if command_off_sim is not None and
                  r['sim_stamp_s'] >= command_off_sim]
    stopping_path = sum(math.hypot(b['x_m']-a['x_m'], b['y_m']-a['y_m'])
                        for a,b in zip(after_stop, after_stop[1:]))
    initial_pitch = next((r['pitch_rad'] for r in rows if r['pitch_rad'] is not None), None)
    data.update(variant=args.variant, profile=args.profile,
                config=str(args.payload_config),
                final_distance_m=rows[-1]['distance_m'] if rows else None,
                stopping_distance_m=stopping_path if after_stop else None,
                net_x_after_stop_m=(after_stop[-1]['x_m']-after_stop[0]['x_m']
                                    if after_stop else None),
                max_pitch_change_rad=(max(abs(r['pitch_rad']-initial_pitch)
                                       for r in rows if r['pitch_rad'] is not None)
                                      if initial_pitch is not None else None),
                yaw_change_rad=(math.atan2(
                    math.sin(rows[-1]['yaw_rad']-rows[0]['yaw_rad']),
                    math.cos(rows[-1]['yaw_rad']-rows[0]['yaw_rad'])) if rows else None),
                observed_hazard_types=sorted(hazard_types),
                hazard_events=hazard_events,
                elapsed_sim_s=rows[-1]['sim_stamp_s']-sim_start if rows and sim_start else None,
                elapsed_wall_s=time.monotonic()-wall_start,
                scope='Open-loop physical response only; Nav2 safety is not in this trial')
    (args.output_dir/'summary.json').write_text(json.dumps(data, indent=2))
    if rows:
        with (args.output_dir/'motion.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(data, indent=2), flush=True)


if __name__ == '__main__':
    main()
