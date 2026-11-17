"""File-only odom/IMU observer: no odometry/TF/command publishers."""
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
from osp_mission.pose_gyro_stream import PoseGyroStream


def rotation(q):
    x,y,z,w=np.asarray(q,float);norm=x*x+y*y+z*z+w*w
    if not np.isfinite([x,y,z,w]).all() or abs(norm-1.)>.01:raise ValueError('Invalid IMU extrinsics')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=Node('pose_gyro_file_observer')
    for name,value in [('output_dir','/tmp/pose_gyro_shadow'),('gyro_bias_z',0.)]:node.declare_parameter(name,value)
    output=Path(node.get_parameter('output_dir').value);output.mkdir(parents=True,exist_ok=True)
    bias=float(node.get_parameter('gyro_bias_z').value)
    if not math.isfinite(bias):raise ValueError('Invalid explicit gyro bias')
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    stream=PoseGyroStream();mount=None;imu_frame=None;outputs=0;last_write=0.;tf_waits=0
    counts=dict(odom=0,imu=0)
    log=(output/'pose_gyro_shadow.csv').open('x',newline='');writer=csv.writer(log)
    writer.writerow(['stamp','x','y','yaw']);started=time.monotonic()
    def save(finished=False):
        log.flush()
        report=dict(file_only=True,tf_published=False,commands_published=False,ground_truth_input=False,
            hardware_calibrated=False,explicit_gyro_bias_z=bias,finished=finished,counts=counts,outputs=outputs,
            imu_frame=imu_frame,base_from_imu_rotation=mount.tolist() if mount is not None else None,
            tf_waits=tf_waits,fault=stream.fault,duplicates_removed=stream.duplicates,
            wall_seconds=time.monotonic()-started)
        temporary=output/'pose_gyro_shadow.tmp';temporary.write_text(json.dumps(report,indent=2)+'\n')
        temporary.replace(output/'pose_gyro_shadow.json')
    def receive(m,sensor):
        nonlocal mount,imu_frame,outputs,last_write,tf_waits
        counts[sensor]+=1
        if stream.fault:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        try:
            if sensor=='imu':
                if imu_frame is not None and m.header.frame_id!=imu_frame:raise ValueError('IMU frame changed')
                if mount is None:
                    try:
                        tr=buffer.lookup_transform('base_link',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                    except Exception:tf_waits+=1;return
                    q=tr.rotation;mount=rotation([q.x,q.y,q.z,q.w]);imu_frame=m.header.frame_id
                vector=mount@np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
                if not np.isfinite(vector).all():raise ValueError('Nonfinite base gyro')
                rows=stream.imu(stamp,float(vector[2])-bias)
            else:
                # Wheel collection starts only once the IMU mounting was observed.
                if mount is None:return
                if m.header.frame_id!='odom' or m.child_frame_id!='base_link':raise ValueError('Unexpected wheel frames')
                p=m.pose.pose.position;q=m.pose.pose.orientation
                if not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:
                    raise ValueError('Invalid wheel quaternion')
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                rows=stream.wheel([stamp,p.x,p.y,yaw])
            for row in rows:writer.writerow(row);outputs+=1
        except Exception as exc:stream.fault=str(exc);node.get_logger().error('Shadow stopped: '+str(exc));save()
        if time.monotonic()-last_write>2.:save();last_write=time.monotonic()
    subscriptions=[node.create_subscription(Odometry,'odom',lambda m:receive(m,'odom'),qos_profile_sensor_data),
        node.create_subscription(Imu,'imu',lambda m:receive(m,'imu'),qos_profile_sensor_data)]
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        save(finished=True);log.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
