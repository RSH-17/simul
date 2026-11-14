"""Read-only verification that the observed fixed dock reaches actual Nav2 maps."""
import argparse
import json
import math
from pathlib import Path
import shlex
import subprocess
import time

import numpy as np


def gate(report):
    errors=[]
    for name in ['local_costmap','global_costmap']:
        if report.get('parameters',{}).get(name)!='mission/navigation_map':errors.append(name+' map topic')
        if '/'+name+'/'+name not in report.get('subscribers',[]):errors.append(name+' actual subscription')
    maps=report.get('paired_maps',{})
    for field in ['same_metadata','only_dock_occupied_changes','dock_center_occupied']:
        if maps.get(field) is not True:errors.append(field)
    if maps.get('changed_cells',0)<1:errors.append('no known geometry added')
    if report.get('anchor_samples',0)<2 or report.get('anchor_constant') is not True:errors.append('stable observed map anchor')
    if report.get('global_dock_cost')!=254:errors.append('dock absent from actual global lethal costs')
    if not isinstance(report.get('local_dock_in_bounds'),bool):errors.append('actual local map unavailable')
    if report.get('local_dock_in_bounds') is True and report.get('local_dock_cost')!=254:
        errors.append('dock absent from actual local lethal costs')
    return dict(passed=not errors,errors=errors)


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/env','ROS_LOCALHOST_ONLY=0','ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
        'FASTDDS_BUILTIN_TRANSPORTS=UDPv4','/usr/bin/python3',str(Path(__file__).resolve()),
        '--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],check=True,capture_output=True,text=True,timeout=40)
    return json.loads(output.read_text())


def measure(domain):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    rclpy.init(domain_id=domain)
    node=rclpy.create_node('dock_map_readonly_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps={'map':{},'navigation':{}};costs={};anchors=[];subscriptions=[];pending={};clients=[]
    def receive_map(name,m):
        key=(m.header.stamp.sec,m.header.stamp.nanosec,m.info.width,m.info.height)
        maps[name][key]=m
        if len(maps[name])>12:del maps[name][next(iter(maps[name]))]
    def status(m):
        a=json.loads(m.data).get('dock_anchor',{})
        if a.get('status')=='captured_before_undock':anchors.append(a)
    for key,topic in [('map','map'),('navigation','mission/navigation_map')]:
        subscriptions.append(node.create_subscription(OccupancyGrid,topic,lambda m,k=key:receive_map(k,m),durable))
    subscriptions.append(node.create_subscription(String,'mission/status',status,10))
    for key in ['local_costmap','global_costmap']:
        subscriptions.append(node.create_subscription(Costmap,key+'/costmap_raw',lambda m,k=key:costs.update({k:m}),qos_profile_sensor_data))
        client=node.create_client(GetParameters,key+'/'+key+'/get_parameters');clients.append(client)
        if client.wait_for_service(timeout_sec=1.):
            req=GetParameters.Request();req.names=['static_layer.map_topic'];pending[key]=client.call_async(req)
    end=time.monotonic()+20
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    report=dict(read_only=True,ground_truth_used=False,parameters={},anchor_samples=len(anchors),
        anchor_constant=bool(anchors) and all(a['pose']==anchors[0]['pose'] for a in anchors),
        subscribers=sorted(i.node_namespace.rstrip('/')+'/'+i.node_name
                           for i in node.get_subscriptions_info_by_topic('mission/navigation_map')))
    for name,future in pending.items():
        if future.done() and future.result():report['parameters'][name]=future.result().values[0].string_value
    if anchors:
        anchor=anchors[0];x,y,yaw=anchor['pose'];center=np.array([x-.045*math.cos(yaw),y-.045*math.sin(yaw)])
        report['anchor']=anchor
        common=maps['map'].keys()&maps['navigation'].keys()
        if common:
            key=max(common);a=maps['map'][key];b=maps['navigation'][key]
            raw=np.array(a.data).reshape(a.info.height,a.info.width);nav=np.array(b.data).reshape(b.info.height,b.info.width)
            rows,cols=np.indices(raw.shape);r=a.info.resolution;o=a.info.origin
            q=o.orientation;map_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            gx=(cols+.5)*r;gy=(rows+.5)*r
            wx=o.position.x+gx*math.cos(map_yaw)-gy*math.sin(map_yaw)
            wy=o.position.y+gx*math.sin(map_yaw)+gy*math.cos(map_yaw)
            dx=wx-x;dy=wy-y;lx=dx*math.cos(yaw)+dy*math.sin(yaw);ly=-dx*math.sin(yaw)+dy*math.cos(yaw)
            footprint=(lx>=-.095-r/2)&(lx<=.005+r/2)&(abs(ly)<=.095+r/2)
            delta=center-[o.position.x,o.position.y];c=math.cos(map_yaw);s=math.sin(map_yaw)
            changed=nav!=raw;cx,cy=np.floor((np.array([[c,s],[-s,c]])@delta)/r).astype(int)
            center_value=int(nav[cy,cx]) if 0<=cy<nav.shape[0] and 0<=cx<nav.shape[1] else None
            report['paired_maps']=dict(same_metadata=a.info==b.info and a.header==b.header and a.header.frame_id==anchor['frame'],
                only_dock_occupied_changes=bool(np.all((~changed)|(footprint&(nav==100)))),
                changed_cells=int(changed.sum()),dock_center_occupied=center_value==100,
                observed_center_value=int(raw[cy,cx]) if center_value is not None else None)
        for name,m in costs.items():
            try:
                point=center.copy()
                if m.header.frame_id!=anchor['frame']:
                    t=buffer.lookup_transform(m.header.frame_id,anchor['frame'],Time.from_msg(m.header.stamp)).transform
                    q=t.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    point=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])@point+[t.translation.x,t.translation.y]
                meta=m.metadata;ox,oy=meta.origin.position.x,meta.origin.position.y
                cx,cy=np.floor((point-[ox,oy])/meta.resolution).astype(int)
                inside=0<=cx<meta.size_x and 0<=cy<meta.size_y;prefix=name.split('_')[0]
                report[prefix+'_dock_in_bounds']=bool(inside)
                if inside:report[prefix+'_dock_cost']=int(m.data[cy*meta.size_x+cx])
            except Exception as exc:report[name+'_error']=str(exc)
    node.destroy_node();rclpy.shutdown();report['gate']=gate(report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(measure(a.domain),indent=2)+'\n')
