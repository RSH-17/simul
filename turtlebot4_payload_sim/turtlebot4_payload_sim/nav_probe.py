"""Launch one payload variant, send the same Nav2 goal, and log observed motion."""
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
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from turtlebot4_payload_sim.model import VARIANTS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', choices=VARIANTS, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--goal-x', type=float, default=.5)
    parser.add_argument('--goal-y', type=float, default=0.)
    parser.add_argument('--timeout-wall', type=float, default=150.)
    parser.add_argument('--payload-config', type=Path)
    parser.add_argument('--fixture-filter', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.domain <= 230:
        parser.error('domain must be 0..230')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                       GZ_PARTITION=f'payload_nav_{args.variant}_{os.getpid()}')
    log = (args.output_dir/'launch.log').open('w')
    command = ['ros2','launch','turtlebot4_payload_sim','payload_sim.launch.py',
               f'variant:={args.variant}','navigation:=true']
    if args.payload_config:
        command.append(f'payload_config:={args.payload_config}')
    if args.fixture_filter:
        command.append('fixture_filter:=true')
    process = subprocess.Popen(
        command,
        env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = rclpy.create_node(f'payload_nav_probe_{args.variant}',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    data = {'map_width_m': 0., 'map_height_m': 0., 'costmap_lethal': None,
            'costmap_inflated': None, 'collision_action': None,
            'commanded_linear_max_mps': 0., 'actual_linear_max_mps': 0.,
            'first_motion_wall_s': None, 'goal_sent_wall_s': None,
            'first_motion_sim_s': None, 'goal_sent_sim_s': None,
            'goal_completion_sim_s': None,
            'max_abs_roll_rad': 0., 'max_abs_pitch_rad': 0.,
            'max_horizontal_accel_mps2': 0., 'max_odom_accel_mps2': 0.,
            'goal_accepted': None, 'goal_status': None}
    rows = []
    start = time.monotonic()
    first_odom = None
    previous_odom = None

    def on_map(message):
        data['map_width_m'] = message.info.width*message.info.resolution
        data['map_height_m'] = message.info.height*message.info.resolution

    def on_costmap(message):
        data['costmap_lethal'] = sum(v >= 254 for v in message.data)
        data['costmap_inflated'] = sum(0 < v < 254 for v in message.data)

    def on_collision(message):
        data['collision_action'] = int(message.action_type)

    def on_command(message):
        data['commanded_linear_max_mps'] = max(
            data['commanded_linear_max_mps'], abs(message.twist.linear.x))

    def on_odom(message):
        nonlocal first_odom, previous_odom
        p = message.pose.pose.position
        if first_odom is None:
            first_odom = (p.x, p.y)
        distance = math.hypot(p.x-first_odom[0], p.y-first_odom[1])
        velocity = message.twist.twist.linear.x
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        data['actual_linear_max_mps'] = max(data['actual_linear_max_mps'], abs(velocity))
        if data['first_motion_wall_s'] is None and distance > .02:
            data['first_motion_wall_s'] = time.monotonic()-start
            data['first_motion_sim_s'] = stamp
        if previous_odom is not None and stamp > previous_odom[0]:
            acceleration = abs(velocity-previous_odom[1])/(stamp-previous_odom[0])
            data['max_odom_accel_mps2'] = max(data['max_odom_accel_mps2'], acceleration)
        previous_odom = (stamp, velocity)
        rows.append({'wall_elapsed_s': round(time.monotonic()-start, 3),
                     'sim_stamp_s': stamp,
                     'odom_x_m': p.x, 'odom_y_m': p.y,
                     'distance_from_start_m': distance, 'actual_linear_mps': velocity,
                     'collision_action': data['collision_action']})

    def on_imu(message):
        q = message.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        data['max_abs_roll_rad'] = max(data['max_abs_roll_rad'], abs(roll))
        data['max_abs_pitch_rad'] = max(data['max_abs_pitch_rad'], abs(pitch))
        a = message.linear_acceleration
        data['max_horizontal_accel_mps2'] = max(
            data['max_horizontal_accel_mps2'], math.hypot(a.x, a.y))

    node.create_subscription(OccupancyGrid, '/map', on_map, qos)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', on_costmap, qos)
    node.create_subscription(CollisionMonitorState, '/collision_monitor_state', on_collision, 10)
    node.create_subscription(TwistStamped, '/cmd_vel', on_command, 10)
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    node.create_subscription(Imu, '/imu', on_imu, 10)
    sent = None
    result = None
    state_future = None
    last_state_query = 0.
    navigator_active = False
    try:
        while time.monotonic()-start < args.timeout_wall:
            if process.poll() is not None:
                data['reason'] = 'simulator_exited'
                break
            rclpy.spin_once(node, timeout_sec=.25)
            ready_map = min(data['map_width_m'], data['map_height_m']) >= 3.
            if lifecycle.service_is_ready() and not navigator_active:
                if state_future is not None and state_future.done():
                    navigator_active = state_future.result().current_state.id == 3
                    state_future = None
                if state_future is None and time.monotonic()-last_state_query > 2.:
                    state_future = lifecycle.call_async(GetState.Request())
                    last_state_query = time.monotonic()
            if ready_map and navigator_active and sent is None and client.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = node.get_clock().now().to_msg()
                goal.pose.pose.position.x = args.goal_x
                goal.pose.pose.position.y = args.goal_y
                goal.pose.pose.orientation.w = 1.
                sent = client.send_goal_async(goal)
                data['goal_sent_wall_s'] = time.monotonic()-start
                data['goal_sent_sim_s'] = node.get_clock().now().nanoseconds/1e9
            if sent is not None and sent.done() and result is None:
                handle = sent.result()
                data['goal_accepted'] = bool(handle.accepted)
                if not handle.accepted:
                    data['reason'] = 'goal_rejected'
                    break
                result = handle.get_result_async()
            if result is not None and result.done():
                data['goal_status'] = int(result.result().status)
                data['goal_completion_sim_s'] = node.get_clock().now().nanoseconds/1e9
                data['reason'] = 'goal_result'
                break
        else:
            data['reason'] = 'wall_timeout'
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
    data.update(variant=args.variant, goal_map_m=[args.goal_x, args.goal_y],
                payload_config=str(args.payload_config) if args.payload_config else 'default',
                fixture_filter=args.fixture_filter,
                wall_seconds=round(time.monotonic()-start, 3),
                scope='One Gazebo run on a VM; no physical-robot conclusion')
    (args.output_dir/'summary.json').write_text(json.dumps(data, indent=2))
    if rows:
        with (args.output_dir/'odom.csv').open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(data, indent=2), flush=True)


if __name__ == '__main__':
    main()
