"""Isolated ROS-only map/TF fixture through the installed Nav2 StaticLayer.

No Gazebo or robot command publishers. This is wiring/coordinate integration,
not evidence of native mission success. The caller supplies an unused domain.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import yaml


def verify(workspace,output,domain):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from tf2_ros import TransformBroadcaster
    from osp_mission.dock_geometry import navigation_map,navigation_map_remappings
    namespace='/dock_test_tb4';rclpy.init(domain_id=domain)
    node=rclpy.create_node('dock_map_fixture',namespace=namespace,cli_args=['--ros-args','-r','/tf:=tf'])
    tf=TransformBroadcaster(node)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(OccupancyGrid,'mission/navigation_map',qos)
    original=OccupancyGrid();original.header.frame_id='map';original.info.width=160;original.info.height=160
    original.info.resolution=.025;original.info.origin.position.x=-2.;original.info.origin.position.y=-2.
    original.info.origin.orientation.w=1.;original.data=[0]*25600
    state=dict(overlay=False,correction=0.);latest={};processes=[];streams=[]
    def tick():
        stamp=node.get_clock().now().to_msg();frames=[]
        for parent,child,x in [('map','odom',state['correction']),('odom','base_link',-.5-state['correction'])]:
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x=x;t.transform.rotation.w=1.;frames.append(t)
        tf.sendTransform(frames);original.header.stamp=stamp
        pub.publish(navigation_map(original,(.157,0.,math.pi) if state['overlay'] else None))
    timer=node.create_timer(.1,tick)
    profile=yaml.safe_load((workspace/'src/osp_mission/config/tb4_nav2.yaml').read_text())
    def spin_until(check,seconds=12):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if check():return
        raise TimeoutError('ROS fixture timed out: '+str(node.get_node_names_and_namespaces()))
    def cell(name,x,y=0.):
        m=latest.get(name)
        if m is None:return None
        meta=m.metadata;cx=int(math.floor((x-meta.origin.position.x)/meta.resolution));cy=int(math.floor((y-meta.origin.position.y)/meta.resolution))
        return int(m.data[cy*meta.size_x+cx]) if 0<=cx<meta.size_x and 0<=cy<meta.size_y else None
    try:
        with tempfile.TemporaryDirectory(prefix='osp_dock_ros_') as temp:
            clients=[];subscriptions=[]
            for name in ['local_costmap','global_costmap']:
                params=copy.deepcopy(profile[name][name]['ros__parameters']);params['use_sim_time']=False
                path=Path(temp)/(name+'.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
                argv=['/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d','--ros-args','--params-file',str(path),
                    '-r','__node:='+name,'-r','__ns:='+namespace+'/'+name,'-r','/tf:='+namespace+'/tf',
                    '-r','/tf_static:='+namespace+'/tf_static']
                for source,target in navigation_map_remappings(namespace):argv+=['-r',source+':='+target]
                log=output.with_name(output.stem+'_'+name+'.log').open('w');streams.append(log)
                processes.append(subprocess.Popen(argv,env=dict(os.environ,ROS_DOMAIN_ID=str(domain),
                    ROS_LOCALHOST_ONLY='0',ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET',FASTDDS_BUILTIN_TRANSPORTS='UDPv4'),stdout=log,stderr=log))
                subscriptions.append(node.create_subscription(Costmap,name+'/costmap_raw',lambda m,k=name:latest.update({k:m}),qos_profile_sensor_data))
                client=node.create_client(ChangeState,name+'/'+name+'/change_state');clients.append(client)
                spin_until(client.service_is_ready)
                for transition in [1,3]:
                    request=ChangeState.Request();request.transition.id=transition;future=client.call_async(request)
                    spin_until(future.done);assert future.result().success
            spin_until(lambda:all(cell(n,.202)==0 for n in ['local_costmap','global_costmap']))
            before={n:cell(n,.202) for n in latest}
            state['overlay']=True
            spin_until(lambda:all(cell(n,.202)==254 for n in ['local_costmap','global_costmap']))
            marked={n:cell(n,.202) for n in latest}
            state['correction']=.35
            spin_until(lambda:cell('local_costmap',.202-.35)==254 and cell('local_costmap',.202)!=254
                       and cell('global_costmap',.202)==254)
            result=dict(evidence_type='ROS-only synthetic map/TF integration through installed Nav2; not Gazebo',
                domain=domain,namespace=namespace,free_before=before,lethal_after=marked,
                after_map_odom_translation_m=.35,local_corrected_dock_cost=cell('local_costmap',-.148),
                local_old_dock_cell_cost=cell('local_costmap',.202),global_fixed_dock_cost=cell('global_costmap',.202),
                source_map_unchanged=all(v==0 for v in original.data),robot_commands_published=False,passed=True)
            output.write_text(json.dumps(result,indent=2)+'\n');return result
    finally:
        for process in processes:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,required=True);a=p.parse_args()
    print(json.dumps(verify(a.workspace.resolve(),a.output.resolve(),a.domain)))
