"""Matched camera-profile launch for frozen v1.2 and camera-aware v1.5."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable,TimerAction,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(path,**kwargs):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=kwargs.items())


def render_backend(environment=os.environ):
    """`software` (default) forces Mesa llvmpipe as in the original campaigns.

    `system` uses the host GPU driver. Use it only where software rendering
    fails (e.g. NVIDIA hosts whose headless EGL device crashes Mesa), and
    compare runs only with others recorded under the same backend."""
    value=environment.get('TEAM_SIM_RENDER','software').strip().lower()
    if value not in ('software','system'):raise ValueError('TEAM_SIM_RENDER must be software or system')
    return value


def generate_launch_description():
    root=Path(os.environ['BENCH_ROOT']);run=Path(os.environ['BENCH_RUN']);flow=Path(share('osp_mission'))
    expected=os.environ['CAMERA_POLICY_WORKSPACE']
    assert str(flow).startswith(str(Path(expected)/'install')), (flow,expected)
    cfg=yaml.safe_load((root/'scene.yaml').read_text());x,y,yaw=map(str,cfg['start'])
    resources=':'.join(str(Path(share(p)).parent) for p in ('turtlebot4_description','irobot_create_description'))
    simulation=include(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py',
                       gz_args=f'{root}/world/osp_studio.sdf -r -v 2 -s --headless-rendering')
    spawn=include(flow/'launch/spawn.launch.py',world='osp_studio',model='standard',namespace='',
                  x=x,y=y,yaw=yaw,camera_width='640',start_docked='false',rviz='false')
    random_exam=os.environ.get('TEAM_RANDOM_EXAM')=='1'
    detector={'rgb_topic':'/exam/detector_rgb'} if random_exam else {}
    mission=include(flow/'launch/robot.launch.py',startup_guard='true',use_sim_time='true',
        require_drive_controller='true',undock='false',rviz='false',planner='edge',
        duration=str(cfg['duration']),navigation_flow_enabled='true',allow_mid_route_wall_inspection='false' if random_exam else 'true',
        planner_start_ignore_radius_m='0.30',depth_topic='unused_depth',
        policy_file=str(Path(expected)/'frozen-config/policy.json'),
        nav2_params=str(Path(expected)/'frozen-config/nav2_hardware_profile.yaml'),output_dir=str(run/'native'),**detector)
    return LaunchDescription([DeclareLaunchArgument('namespace',default_value=''),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        *([SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1')] if render_backend()=='software' else []),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),SetEnvironmentVariable('GZ_PARTITION','camera_aware_'+str(os.getpid())),
        simulation,Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='ros_gz_bridge',executable='parameter_bridge',
             arguments=['/world/osp_studio/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/world/osp_studio/pose/info','/_internal/sim_ground_truth_pose')]),
        TimerAction(period=3.,actions=[spawn]),TimerAction(period=6.,actions=[mission])])
