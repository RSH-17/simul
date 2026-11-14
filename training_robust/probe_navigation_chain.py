"""Read-only maps, command chain, footprint and parameters in one live domain.

Snapshots are received asynchronously, not a synchronized sensor replay. No
commands, goals or parameter changes are sent to a running experiment.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np


def measure(domain, output, seconds=12.):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from rcl_interfaces.srv import GetParameters
    from rosidl_runtime_py.convert import message_to_ordereddict
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from tf2_ros import Buffer, TransformListener
    from probe_navigation_deadlock import yaw

    rclpy.init(domain_id=domain)
    node=Node('navigation_chain_readonly',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    output.mkdir(parents=True,exist_ok=False)
    latest={};counts={};series={};subscriptions=[];clients=[];pending=[]
    def receive(topic,message):
        latest[topic]=message;counts[topic]=counts.get(topic,0)+1
        if hasattr(message,'twist') and hasattr(message.twist,'linear'):
            now=node.get_clock().now().nanoseconds/1e9
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9 if hasattr(message,'header') else None
            series.setdefault(topic,[]).append([now,stamp,message.twist.linear.x,message.twist.angular.z])
    durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,typ,qos in [('map',OccupancyGrid,durable),('local_costmap/costmap_raw',Costmap,qos_profile_sensor_data),
                          ('global_costmap/costmap_raw',Costmap,qos_profile_sensor_data)]:
        subscriptions.append(node.create_subscription(typ,topic,lambda m,t=topic:receive(t,m),qos))
    for topic,names in [('local_costmap/local_costmap',['resolution','footprint','footprint_padding','robot_radius','plugins']),
                        ('global_costmap/global_costmap',['resolution','footprint','footprint_padding']),
                        ('controller_server',['FollowPath.CostCritic.consider_footprint','FollowPath.CostCritic.collision_cost','FollowPath.vx_max']),
                        ('collision_monitor',['FootprintApproach.time_before_collision','FootprintApproach.min_points'])]:
        client=node.create_client(GetParameters,topic+'/get_parameters');clients.append(client)
        if client.wait_for_service(timeout_sec=1.):
            request=GetParameters.Request();request.names=names;pending.append((topic,names,client.call_async(request)))
    requested=['cmd_vel_nav','cmd_vel_smoothed','cmd_vel','diffdrive_controller/cmd_vel',
               'collision_monitor_state','local_costmap/published_footprint','plan','local_plan','mission/status']
    graph=dict(node.get_topic_names_and_types());subscribed=[]
    for topic in requested:
        types=graph.get('/'+topic,[])
        if len(types)!=1:continue
        subscriptions.append(node.create_subscription(get_message(types[0]),topic,
            lambda m,t=topic:receive(t,m),qos_profile_sensor_data));subscribed.append(topic)
    started=time.monotonic()
    while time.monotonic()-started<seconds:rclpy.spin_once(node,timeout_sec=.1)
    report=dict(domain=domain,read_only=True,commands_published=False,
        scope='Asynchronous actual ROS inputs; no ground truth or parameter changes',
        subscribed=subscribed,received=counts,parameters={},command_samples=series,latest={},transforms={})
    for topic,names,future in pending:
        if future.done() and future.result() is not None:
            report['parameters'][topic]={name:message_to_ordereddict(value) for name,value in zip(names,future.result().values)}
    for topic,message in latest.items():
        if isinstance(message,(OccupancyGrid,Costmap)):
            meta=message.info if isinstance(message,OccupancyGrid) else message.metadata
            width=meta.width if isinstance(message,OccupancyGrid) else meta.size_x
            height=meta.height if isinstance(message,OccupancyGrid) else meta.size_y
            origin=[meta.origin.position.x,meta.origin.position.y,yaw(meta.origin.orientation)]
            name=topic.replace('/','_')+'.npz'
            np.savez_compressed(output/name,data=np.asarray(message.data).reshape(height,width),
                resolution=meta.resolution,origin=origin)
            report['latest'][topic]=dict(file=name,resolution=meta.resolution,frame=message.header.frame_id,
                stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9)
        else:
            report['latest'][topic]=message_to_ordereddict(message)
    for parent,child in [('map','base_link'),('odom','base_link'),('map','odom')]:
        try:report['transforms'][parent+'<-'+child]=message_to_ordereddict(buffer.lookup_transform(parent,child,Time()))
        except Exception as exc:report['transforms'][parent+'<-'+child]=dict(error=str(exc))
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
    return dict(received=counts,command_ranges={k:dict(samples=len(v),linear_min=min(x[2] for x in v),
        linear_max=max(x[2] for x in v),angular_min=min(x[3] for x in v),angular_max=max(x[3] for x in v)) for k,v in series.items()})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=12.)
    a=p.parse_args();print(json.dumps(measure(a.domain,a.output.resolve(),a.seconds),indent=2))
