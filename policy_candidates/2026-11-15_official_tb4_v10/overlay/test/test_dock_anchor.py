"""Known charging station must remain a fixed obstacle after odom correction."""
import math
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

from osp_mission import dock_geometry
from osp_mission.mission import Mission


def transform(x=.157,y=0.,yaw=math.pi):
    t=TransformStamped();t.transform.translation.x=x;t.transform.translation.y=y
    t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
    return t


def owner():
    node=S(p=lambda key:'map',now=lambda:3.,
        tf=S(lookup_transform=lambda *a,**kw:transform()),undock_diagnostics={},dock_anchor=None)
    node.known_dock_pose=lambda frame:Mission.known_dock_pose(node,frame)
    return node


def test_anchor_does_not_follow_later_odom_correction():
    node=owner();assert Mission.capture_dock_anchor(node)
    assert node.known_dock_pose('map') is None  # do not obstruct autonomous undock
    node.undock_diagnostics={'status':4,'is_docked':False}
    node.tf.lookup_transform=lambda *a,**kw:transform(.6,-.4,2.)
    assert node.known_dock_pose('map')==pytest.approx((.157,0.,math.pi))
    assert node.known_dock_pose('odom') is None
    node.undock_diagnostics['is_docked']=True
    assert node.known_dock_pose('map') is None


@pytest.mark.parametrize('failure',['missing','nan'])
def test_unavailable_dock_is_not_replaced_by_an_assumed_origin(failure):
    node=owner()
    def fail(*a,**kw):
        if failure=='nan':return transform(float('nan'))
        raise LookupError('no observed dock frame')
    node.tf.lookup_transform=fail
    assert not Mission.capture_dock_anchor(node)
    assert node.dock_anchor is None and node.dock_anchor_diagnostics['status']=='unavailable'


def test_missing_dock_anchor_does_not_start_wheel_motion():
    actions=[]
    node=S(state='idle',ready=lambda:True,p=lambda key:True,dock_status={'stamp':3.,'is_docked':True},
        sensor_fresh=lambda s:True,undock=S(server_is_ready=lambda:True),capture_dock_anchor=lambda:False,
        send=lambda *a:actions.append(a))
    response=S(success=False,message='')
    Mission.start(node,None,response)
    assert not response.success and not actions and node.state=='idle'
    assert 'dock map transform' in response.message


def grid(yaw=0.):
    m=OccupancyGrid();m.header.frame_id='map';m.info.width=100;m.info.height=100;m.info.resolution=.02
    m.info.origin.position.x=-1.;m.info.origin.position.y=-1.
    m.info.origin.orientation.z=math.sin(yaw/2);m.info.origin.orientation.w=math.cos(yaw/2)
    data=np.zeros((100,100),dtype=np.int8);data[70:,:]=-1;data[:,80]=100
    m.data=data.ravel().tolist();return m


def test_navigation_map_adds_only_known_occupied_cells_and_preserves_slam():
    source=grid();before=np.array(source.data).copy();pose=(.157,0.,math.pi)
    result=dock_geometry.navigation_map(source,pose);after=np.array(result.data)
    changed=after!=before
    assert changed.any() and np.all(after[changed]==100)
    assert np.array_equal(source.data,before)
    assert after.reshape(100,100)[50,60]==100  # physical dock center (.202, 0)
    assert after.reshape(100,100)[50,25]==0   # existing escape stays open
    assert result.header==source.header and result.info==source.info
    assert dock_geometry.navigation_map(source,None) is source


def test_rotated_map_origin_and_cloud_use_the_same_station_geometry():
    source=grid(math.pi/2);pose=(-2.,.157,-math.pi/2)
    result=dock_geometry.navigation_map(source,pose)
    data=np.array(result.data).reshape(100,100)
    # Origin (-1,-1), grid rotation pi/2: dock center (-2,.202) is cell (60,50).
    assert data[50,60]==100
    cloud=dock_geometry.points_in_map(pose)
    assert cloud[:,:2].mean(axis=0)==pytest.approx([-2.,.202])


def test_map_callback_keeps_public_slam_map_and_publishes_dock_for_nav2():
    node=owner();assert Mission.capture_dock_anchor(node)
    node.undock_diagnostics={'status':4,'is_docked':False}
    published=[];node.navigation_map=S(publish=published.append)
    node.publish_navigation_map=lambda:Mission.publish_navigation_map(node)
    source=grid();Mission.on_map(node,source)
    assert node.map is source and published[-1] is not source
    assert np.array(published[-1].data).reshape(100,100)[50,60]==100
    assert np.array(node.map.data).reshape(100,100)[50,60]==0


def test_both_actual_nav2_static_layers_consume_navigation_map():
    config=yaml.safe_load((Path(__file__).parents[1]/'config/tb4_nav2.yaml').read_text())
    for name in ['local_costmap','global_costmap']:
        p=config[name][name]['ros__parameters']
        assert p['static_layer']['map_topic']=='mission/navigation_map'
        assert 'static_layer' in p['plugins']
        assert p['static_layer']['map_subscribe_transient_local'] is True


@pytest.mark.parametrize('namespace,prefix',[('',''),('tb4','/tb4'),('/fleet/tb4/','/fleet/tb4')])
def test_costmap_consumers_resolve_to_the_same_robot_namespace(namespace,prefix):
    routes=dock_geometry.navigation_map_remappings(namespace)
    assert routes==[(prefix+'/'+name+'/mission/navigation_map',prefix+'/mission/navigation_map')
                   for name in ['local_costmap','global_costmap']]
