"""Read-only full-rate wheel/IMU recording; never publishes robot commands."""
import argparse
import csv
import json
import time


def record(domain, seconds, output):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init(domain_id=domain)
    node = Node('odometry_input_recorder')
    counts = dict(odom=0, imu=0)
    with open(output, 'x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['sensor', 'stamp', 'frame', 'child_frame', 'x', 'y',
                         'qx', 'qy', 'qz', 'qw', 'linear_x', 'angular_z'])

        def receive(message, sensor):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if sensor == 'odom':
                p, q, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
                values = [message.child_frame_id, p.x, p.y, q.x, q.y, q.z, q.w,
                          twist.linear.x, twist.angular.z]
            else:
                q = message.orientation
                values = ['', '', '', q.x, q.y, q.z, q.w, '', message.angular_velocity.z]
            writer.writerow([sensor, stamp, message.header.frame_id, *values])
            counts[sensor] += 1

        node.create_subscription(Odometry, 'odom', lambda msg: receive(msg, 'odom'), qos_profile_sensor_data)
        node.create_subscription(Imu, 'imu', lambda msg: receive(msg, 'imu'), qos_profile_sensor_data)
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(dict(domain=domain, counts=counts, wall_seconds=seconds,
                          ground_truth_recorded=False, commands_published=False)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    record(args.domain, args.seconds, args.output)
