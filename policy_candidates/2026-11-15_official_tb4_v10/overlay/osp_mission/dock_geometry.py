"""The known charging station is lower than the robot's horizontal LiDAR."""
import copy
import math
import numpy as np


def points():
    # Upstream Create 3 dock collision: x=-.045, size .04 x .13 x .10.
    # Include 3 cm localization margin. These are a known static keepout,
    # not a simulated sensor observation or an unknown scene obstacle.
    x,y=np.meshgrid(np.linspace(-.095,.005,6),np.linspace(-.095,.095,11))
    return np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.05)))


def mark(data,resolution,origin,pose):
    x,y,a=pose
    rows,cols=np.indices(data.shape)
    dx=origin[0]+(cols+.5)*resolution-x
    dy=origin[1]+(rows+.5)*resolution-y
    local_x=dx*math.cos(a)+dy*math.sin(a)
    local_y=-dx*math.sin(a)+dy*math.cos(a)
    margin=resolution/2
    data[(local_x>=-.095-margin)&(local_x<=.005+margin)&(abs(local_y)<=.095+margin)]=100
    return data


def points_in_map(pose):
    x,y,a=pose;cloud=points()
    rotation=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    cloud[:,:2]=cloud[:,:2]@rotation.T+[x,y]
    return cloud


def navigation_map(observed,pose):
    """A separate Nav2 input with only known dock cells added as occupied.

    Preserve the original SLAM message, including unknown cells outside the
    fixture. The known station is fixed in map, not in the drifting odom frame.
    No scenario, target or evaluator input is used.
    """
    if pose is None:return observed
    result=copy.deepcopy(observed)
    origin=observed.info.origin;q=origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx,dy=pose[0]-origin.position.x,pose[1]-origin.position.y
    local=(dx*math.cos(yaw)+dy*math.sin(yaw),-dx*math.sin(yaw)+dy*math.cos(yaw),pose[2]-yaw)
    data=np.array(observed.data,dtype=np.int8).reshape(observed.info.height,observed.info.width)
    mark(data,observed.info.resolution,(0.,0.),local)
    result.data=data.ravel().tolist()
    return result


def navigation_map_remappings(namespace):
    prefix='/'+namespace.strip('/') if namespace.strip('/') else ''
    target=prefix+'/mission/navigation_map'
    return [(prefix+'/'+name+'/mission/navigation_map',target) for name in ['local_costmap','global_costmap']]
