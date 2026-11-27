"""Observe both the local resolution parameter and actual published costmaps."""
import argparse
import ast
import json
import math
from pathlib import Path
import shlex
import subprocess
import time

import official_no_return_probe as sensor
from official_training_input_probe import configuration_gate as original_gate, motion_gate

FOOTPRINT=[[.189,0.],[.134,-.134],[0.,-.189],[-.134,-.134],[-.189,0.],[-.134,.134],[0.,.189],[.134,.134]]
EXTRA={
    'local_costmap/local_costmap': {'resolution':.025,'footprint_padding':.01,
        'footprint':None,'inflation_layer.inflation_radius':.45,'voxel_layer.scan.marking':True},
    'controller_server': {'FollowPath.CostCritic.consider_footprint':True},
    'collision_monitor': {'FootprintApproach.enabled':True,'FootprintApproach.time_before_collision':1.2}}


def navigation_grid_gate(report):
    observed=report.get('parameters',{});errors=[]
    for node,params in EXTRA.items():
        for name,expected in params.items():
            value=observed.get(node,{}).get(name)
            if name=='footprint':
                try:matched=ast.literal_eval(value)==FOOTPRINT
                except (ValueError,TypeError,SyntaxError):matched=False
            elif isinstance(expected,bool):matched=value is expected
            else:matched=isinstance(value,(int,float)) and not isinstance(value,bool) and math.isclose(value,expected,abs_tol=1e-6)
            if not matched:errors.append(node+'/'+name)
    grid=report.get('published_local_grid',{})
    if (grid.get('samples',0)<2 or not math.isclose(grid.get('resolution',0.),.025,abs_tol=1e-6)
            or grid.get('frame')!='odom' or grid.get('width')!=120 or grid.get('height')!=120):
        errors.append('actual 120x120 local costmap at 2.5 cm in odom')
    return dict(passed=not errors,errors=errors)


def configuration_gate(report,policy,expected_duration=90.,**kwargs):
    check=original_gate(report,policy,expected_duration=expected_duration)
    check['missing'].extend(navigation_grid_gate(report)['errors'])
    check['passed']=not check['missing'] and not check['mismatches']
    return check


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/env','ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET','FASTDDS_BUILTIN_TRANSPORTS=UDPv4',
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,check=True,text=True,timeout=85)
    return json.loads(output.read_text())


def measure(domain):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav2_msgs.msg import Costmap
    sensor.EXPECTED={node:dict(values) for node,values in sensor.EXPECTED.items()}
    for node,values in EXTRA.items():sensor.EXPECTED[node].update(values)
    report=sensor.measure(domain)
    rclpy.init(domain_id=domain);node=rclpy.create_node('local_grid_read_only_observer');received={}
    def receive(m):
        received.update(samples=received.get('samples',0)+1,resolution=m.metadata.resolution,
                        width=m.metadata.size_x,height=m.metadata.size_y,frame=m.header.frame_id)
    sub=node.create_subscription(Costmap,'local_costmap/costmap_raw',receive,qos_profile_sensor_data)
    deadline=time.monotonic()+15
    try:
        while time.monotonic()<deadline and received.get('samples',0)<2:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.destroy_node();rclpy.shutdown()
    report['published_local_grid']=received
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(measure(a.domain),indent=2)+'\n')
