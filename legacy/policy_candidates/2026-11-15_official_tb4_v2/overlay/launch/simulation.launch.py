"""Official TB4 spawn/description/control/bridges with task-world and mission additions."""
from pathlib import Path
import os
import tempfile

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from osp_mission.world import generate
from osp_mission.transport_config import image_environment


def setup(context):
    v=context.launch_configurations;pkg=Path(share('osp_mission'))
    if v['camera_width']!='320' or v['start_docked']!='true':
        raise ValueError('Official TB4 spawn retains its 320x240 camera and dock; use camera_width:=320 start_docked:=true')
    if v['render_backend'] not in ('software','system'):
        raise ValueError('render_backend must be software or system')
    if not v['policy_file']:v['policy_file']=str(pkg/'config/edge_policy.json')
    if not v['nav2_params']:v['nav2_params']=str(pkg/'config/tb4_nav2.yaml')
    recording=v['record_video']=='true'
    world,start=generate(v['scenario'],tempfile.mkdtemp(prefix='osp_official_world_'),recording=recording)
    official=Path(share('turtlebot4_gz_bringup'))
    resources=':'.join(str(Path(share(p)).parent) for p in ['turtlebot4_description','irobot_create_description'])
    # Renderer selection belongs to the host/world. The official URDF is unchanged.
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'{world} -r -v 2 -s --headless-rendering --render-engine-server ogre2'}.items())
    spawn=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(official/'launch/turtlebot4_spawn.launch.py')),
        launch_arguments={'world':'osp_studio','model':'standard','namespace':v['namespace'],
            'x':str(start[0]),'y':str(start[1]),'z':'0.0','yaw':str(start[2]),
            'rviz':'false','use_sim_time':'true','slam':'false','nav2':'false','localization':'false'}.items())
    clock=Node(package='ros_gz_bridge',executable='parameter_bridge',name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    robot_name=(v['namespace'].strip('/')+'/' if v['namespace'].strip('/') else '')+'turtlebot4'
    imu_topic=f'/world/osp_studio/model/{robot_name}/link/imu_link/sensor/imu/imu'
    # The task world already includes Gazebo's standard IMU system. Stock TB4
    # bringup omits this bridge; forward the existing official sensor unchanged.
    imu=Node(package='ros_gz_bridge',executable='parameter_bridge',name='imu_bridge',namespace=v['namespace'],
        parameters=[{'use_sim_time':True}],arguments=[imu_topic+'@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[(imu_topic,'imu')])
    imu_tf=Node(package='tf2_ros',executable='static_transform_publisher',name='imu_sensor_frame',namespace=v['namespace'],
        arguments=['--frame-id','imu_link','--child-frame-id',robot_name+'/imu_link/imu'],
        remappings=[('/tf','tf'),('/tf_static','tf_static')])
    shared={k:v[k] for k in ['namespace','slam_mode','session','posegraph','resume_pose','duration','output_dir','rviz','policy_file','nav2_params','planner']}
    shared.update(startup_guard='true',use_sim_time='true',depth_topic='unused_depth',undock='true',
        require_drive_controller='true',platform_profile=str(pkg/'config/official_tb4_simulator.json'))
    mission=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/robot.launch.py')),launch_arguments=shared.items())
    video=[]
    if recording:
        video=[Node(package='ros_gz_bridge',executable='parameter_bridge',additional_env=image_environment(pkg),
                    arguments=['/mission/overview/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']),
               Node(package='osp_mission',executable='record_video',namespace=v['namespace'],output='screen',
                    additional_env=image_environment(pkg),parameters=[{'use_sim_time':True,'output_dir':str(Path(v['output_dir'])/'video')}],
                    remappings=[('/tf','tf'),('/tf_static','tf_static')])]
    gui=[] if v['headless']=='true' else [ExecuteProcess(cmd=['gz','sim','-g','--gui-config',str(official/'gui/standard/gui.config')],output='screen')]
    renderer=[SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1')] if v['render_backend']=='software' else []
    return [*renderer,SetEnvironmentVariable('GZ_PARTITION','osp_official_'+str(os.getpid())),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),sim,clock,*gui,
        TimerAction(period=3.,actions=[spawn,imu,imu_tf]),TimerAction(period=6.,actions=[mission,*video])]


def generate_launch_description():
    pkg=Path(share('osp_mission'))
    defaults={'scenario':str(pkg/'config/robust_demo.yaml'),'headless':'false','rviz':'true','namespace':'',
        'slam_mode':'async','session':'new','posegraph':'','resume_pose':'0,0,0','duration':'600.0',
        'policy_file':'','planner':'edge','camera_width':'320','nav2_params':'','record_video':'false',
        'render_backend':'software','start_docked':'true','output_dir':str(Path.home()/'simul/results/official_tb4')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
