"""Read the actual simulated actuator endpoints/heartbeat; publish no commands."""
import argparse
import json
from pathlib import Path
import time


def measure(domain, seconds=8.):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(domain_id=domain)
    node=Node('sim_drive_path_auditor');receipts=[]
    node.create_subscription(TwistStamped,'diffdrive_controller/cmd_vel',
        lambda message:receipts.append(time.monotonic()),qos_profile_sensor_data)
    deadline=time.monotonic()+seconds
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        observed=dict(velocity_consumer_nodes=[e.node_name for e in node.get_subscriptions_info_by_topic('cmd_vel')],
            drive_publisher_nodes=[e.node_name for e in node.get_publishers_info_by_topic('diffdrive_controller/cmd_vel')],
            drive_consumer_nodes=[e.node_name for e in node.get_subscriptions_info_by_topic('diffdrive_controller/cmd_vel')],
            samples=len(receipts),last_receipt_age_wall_s=time.monotonic()-receipts[-1] if receipts else None)
        passed=('motion_control' in observed['velocity_consumer_nodes']
            and observed['drive_publisher_nodes'].count('motion_control')==1
            and 'diffdrive_controller' in observed['drive_consumer_nodes']
            and len(receipts)>=3 and observed['last_receipt_age_wall_s']<=1.)
        return dict(passed=passed,observed=observed,commands_published=False,
                    scope='actuator endpoint and receipt evidence, not physical movement proof')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.write_text(json.dumps(measure(args.domain),indent=2)+'\n')
