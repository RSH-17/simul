"""Standalone physical-model and sensor experiment using installed TB4 nodes."""
import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from turtlebot4_payload_sim.model import generate, VARIANTS


def include(path, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=arguments.items())


def setup(context):
    v = context.launch_configurations
    package = Path(share('turtlebot4_payload_sim'))
    standard = Path(share('turtlebot4_description'))/'urdf/standard/turtlebot4.urdf.xacro'
    description, report = generate(standard,
        yaml.safe_load(Path(v['payload_config']).read_text()), v['variant'])
    print('Payload model:', report, flush=True)
    world = v['world']
    world_name = 'payload_empty' if world == str(package/'worlds/payload_empty.sdf') else Path(world).stem
    gz = include(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py',
                 gz_args=f'{world} -r -s --headless-rendering --render-engine-server ogre2 -v 2')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'use_sim_time': True, 'robot_description': description}],
               remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])
    joint_pub = Node(package='joint_state_publisher', executable='joint_state_publisher',
                     parameters=[{'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'turtlebot4', '-x', '0', '-y', '0', '-z', '-0.0025',
                            '-topic', 'robot_description'], output='screen')
    bridge = include(Path(share('turtlebot4_gz_bringup'))/'launch/ros_gz_bridge.launch.py',
                     model='standard', robot_name='turtlebot4', dock_name='standard_dock',
                     namespace='', world=world_name, use_sim_time='true')
    nodes = include(Path(share('turtlebot4_gz_bringup'))/'launch/turtlebot4_nodes.launch.py',
                    model='standard')
    create3 = include(Path(share('irobot_create_common_bringup'))/'launch/create3_nodes.launch.py',
                      namespace='')
    gz_nodes = include(Path(share('irobot_create_gz_bringup'))/'launch/create3_gz_nodes.launch.py',
                       robot_name='turtlebot4', dock_name='standard_dock')
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    imu_topic = f'/world/{world_name}/model/turtlebot4/link/imu_link/sensor/imu/imu'
    imu = Node(package='ros_gz_bridge', executable='parameter_bridge', name='payload_imu_bridge',
               arguments=[imu_topic+'@sensor_msgs/msg/Imu[gz.msgs.IMU'],
               remappings=[(imu_topic, '/imu')])
    laser_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0','0','0','0','0','0','rplidar_link',
                               'turtlebot4/rplidar_link/rplidar'],
                    remappings=[('/tf','tf'),('/tf_static','tf_static')])
    nav = []
    filter_node = []
    if v['fixture_filter'] == 'true':
        filter_node = [Node(package='turtlebot4_payload_sim',
                            executable='payload_fixture_filter', output='screen',
                            arguments=['--config', v['payload_config']])]
    if v['navigation'] == 'true':
        navigation = Path(share('turtlebot4_navigation'))/'launch'
        nav_params = Path(share('turtlebot4_navigation'))/'config/nav2.yaml'
        if v['vm_nav_tuning'] == 'true' or v['fixture_filter'] == 'true':
            data = yaml.safe_load(nav_params.read_text())
            if v['vm_nav_tuning'] == 'true':
                controller = data['controller_server']['ros__parameters']
                controller['controller_frequency'] = 20.0
                controller['FollowPath']['batch_size'] = 200
                controller['FollowPath']['time_steps'] = 28
            if v['fixture_filter'] == 'true':
                def replace_scan_topics(value):
                    if isinstance(value, dict):
                        for key, item in value.items():
                            if key == 'topic' and item == 'scan':
                                value[key] = '/scan_payload_filtered'
                            else:
                                replace_scan_topics(item)
                    elif isinstance(value, list):
                        for item in value:
                            replace_scan_topics(item)
                replace_scan_topics(data)
            # Only runtime cost is tuned. Costmap geometry and collision
            # monitor configuration remain identical across all variants.
            path = Path(tempfile.mkdtemp(prefix='payload_nav2_vm_'))/'nav2.yaml'
            path.write_text(yaml.safe_dump(data))
            nav_params = path
        slam = include(navigation/'slam.launch.py', namespace='', use_sim_time='true')
        if v['fixture_filter'] == 'true':
            slam = GroupAction([SetRemap(src='/scan', dst='/scan_payload_filtered'), slam])
        nav = [slam,
               include(navigation/'nav2.launch.py', namespace='', use_sim_time='true',
                       params_file=str(nav_params))]
    resources = ':'.join(str(Path(share(p)).parent) for p in
                         ('turtlebot4_description', 'irobot_create_description'))
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'), gz, clock, imu,
            TimerAction(period=2., actions=[rsp, joint_pub]),
            TimerAction(period=5., actions=[spawn, bridge, nodes, create3, gz_nodes,
                                             laser_tf, *filter_node, *nav])]


def generate_launch_description():
    package = Path(share('turtlebot4_payload_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('variant', default_value='baseline', choices=VARIANTS),
        DeclareLaunchArgument('payload_config',
                              default_value=str(package/'config/payload.yaml')),
        DeclareLaunchArgument('world', default_value=str(package/'worlds/payload_empty.sdf')),
        DeclareLaunchArgument('navigation', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('vm_nav_tuning', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('fixture_filter', default_value='false', choices=['true','false']),
        OpaqueFunction(function=setup)])
