"""Read actual simulation parameters; no commands, goals or parameter changes."""
import json
import os
from pathlib import Path
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def value(item):
    fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',
            5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    if item.type not in fields:return None
    result=getattr(item,fields[item.type])
    return list(result) if item.type>=5 else result


assert os.environ.get('ROS_DOMAIN_ID')=='174' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init();node=Node('read_only_simulation_parameters')
requests={'/mission':['allow_mid_route_wall_inspection','navigation_flow_enabled'],
          '/controller_server':['FollowPath.plugin','FollowPath.vx_max','FollowPath.wz_max','controller_frequency'],
          '/velocity_smoother':['max_velocity','max_accel','max_decel','smoothing_frequency'],
          '/local_costmap/local_costmap':['footprint','inflation_layer.inflation_radius','update_frequency'],
          '/global_costmap/global_costmap':['footprint','inflation_layer.inflation_radius','update_frequency'],
          '/aruco_detector':['min_marker_pixels','confirmations']}
report={}
try:
    for name,names in requests.items():
        client=node.create_client(GetParameters,name+'/get_parameters')
        if not client.wait_for_service(timeout_sec=2.):report[name]={'unavailable':True};continue
        request=GetParameters.Request();request.names=names
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        report[name]=dict(zip(names,map(value,future.result().values))) if future.done() and future.result() else {'timeout':True}
    report['scope']='read-only parameters of isolated simulation domain 174'
    Path(sys.argv[1]).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
