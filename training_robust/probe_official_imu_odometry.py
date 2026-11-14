"""Run an isolated, non-controlling EKF beside an existing official simulation.

Only encoder forward/lateral velocity and IMU yaw rate enter the estimator.
Absolute IMU orientation, SLAM poses and evaluator truth are never inputs.
The estimator publishes a diagnostic topic, with TF publication disabled.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import time

import yaml


def parameters():
    return dict(use_sim_time=True, frequency=62., sensor_timeout=.2,
        two_d_mode=True, publish_tf=False, publish_acceleration=False,
        print_diagnostics=True, map_frame='map', odom_frame='odom',
        base_link_frame='base_link', world_frame='odom',
        odom0='odom', odom0_config=[False]*6+[True, True, False]+[False]*6,
        odom0_queue_size=50, odom0_differential=False, odom0_relative=False,
        imu0='imu', imu0_config=[False]*11+[True]+[False]*3,
        imu0_queue_size=50, imu0_differential=False, imu0_relative=False,
        use_control=False)


def child_arguments(config):
    # robot_localization constructs a TF broadcaster even with publish_tf=false.
    # Isolate the advertised endpoint as well as disabling actual publication.
    return ['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args',
        '-r','__node:=official_imu_shadow','-r','odometry/filtered:=diagnostics/official_imu_odometry',
        '-r','/tf:=diagnostics/imu_shadow_tf','-r','/tf_static:=diagnostics/imu_shadow_tf_static',
        '--params-file',str(config)]


def record(domain, seconds, output):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    output.mkdir(parents=True, exist_ok=False)
    config=output/'ekf.yaml'
    config.write_text(yaml.safe_dump({'official_imu_shadow':{'ros__parameters':parameters()}}))
    rclpy.init(domain_id=domain)
    node=rclpy.create_node('official_imu_read_only_recorder')
    counts={};metadata={};started=time.monotonic();guard=[];error=None
    static_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    static_mirror=node.create_publisher(TFMessage,'diagnostics/imu_shadow_tf_static',static_qos)
    static_sub=node.create_subscription(TFMessage,'tf_static',static_mirror.publish,static_qos)
    def platform(msg):
        observed=json.loads(msg.data);guard.append(observed.get('passed') is True)
        if len(guard)>100:del guard[:-100]
    platform_sub=node.create_subscription(String,'mission/platform_status',platform,10)
    # Explicit domain also applies to the child; never change the active robot.
    import os
    env=dict(os.environ, ROS_DOMAIN_ID=str(domain))
    with (output/'ekf.log').open('w') as log, (output/'inputs.csv').open('x',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['sensor','stamp','x','y','qx','qy','qz','qw','vx','vy','wz'])
        def receive(msg, sensor):
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if sensor=='imu':
                p=None;q=msg.orientation;twist=None
                values=['','',q.x,q.y,q.z,q.w,'','',msg.angular_velocity.z]
                extra=dict(angular_velocity_covariance=list(msg.angular_velocity_covariance),
                           orientation_covariance=list(msg.orientation_covariance))
            else:
                p=msg.pose.pose.position;q=msg.pose.pose.orientation;twist=msg.twist.twist
                values=[p.x,p.y,q.x,q.y,q.z,q.w,twist.linear.x,twist.linear.y,twist.angular.z]
                extra=dict(child_frame=msg.child_frame_id,
                           twist_covariance=list(msg.twist.covariance))
            writer.writerow([sensor,stamp,*values]);counts[sensor]=counts.get(sensor,0)+1
            metadata[sensor]=dict(frame=msg.header.frame_id,**extra)
        subscriptions=[node.create_subscription(cls,topic,lambda m,k=key:receive(m,k),qos_profile_sensor_data)
            for key,topic,cls in [('wheel','odom',Odometry),('imu','imu',Imu),
                ('ekf','diagnostics/official_imu_odometry',Odometry)]]
        if node.count_publishers('diagnostics/official_imu_odometry'):
            raise RuntimeError('Diagnostic output already has a publisher; choose an independent run')
        child=subprocess.Popen(child_arguments(config),env=env,stdout=log,stderr=log)
        try:
            while time.monotonic()-started<seconds:
                if child.poll() is not None:raise RuntimeError('Shadow estimator exited; inspect ekf.log')
                rclpy.spin_once(node,timeout_sec=.1)
                if guard and not guard[-1]:raise RuntimeError('Platform guard failed during observation; discard comparison')
                if any(e.node_name=='official_imu_shadow' for e in node.get_publishers_info_by_topic('tf')):
                    raise RuntimeError('Shadow estimator advertised a production TF endpoint')
                stream.flush()
        except Exception as exc:
            error=str(exc)
        finally:
            child.terminate()
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=5)
            node.destroy_node();rclpy.shutdown()
    report=dict(domain=domain,wall_seconds=time.monotonic()-started,counts=counts,metadata=metadata,
        parameters=parameters(),ground_truth_input=False,absolute_imu_orientation_fused=False,
        tf_published=False,commands_published=False,policy_consumes_output=False,
        tf_endpoints_isolated=True,platform_checks=len(guard),platform_checks_passed=bool(guard) and all(guard),
        error=error,complete=error is None,
        evidence_type='Live shadow estimator only; does not change navigation or qualify task success')
    (output/'recording.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    if error:raise RuntimeError(error)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--seconds',type=float,default=180.);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();record(a.domain,a.seconds,a.output.resolve())
