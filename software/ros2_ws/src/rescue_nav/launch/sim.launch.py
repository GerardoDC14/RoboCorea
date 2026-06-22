"""Gazebo Classic simulation: track world + the sim robot + /scan_flat.

Stands in for the robot hardware so the dicerox_mapping SLAM stack and the Nav2
stack can run unchanged against it:
  * gzserver/gzclient with worlds/track.world
  * robot_state_publisher (robot_sim.urdf.xacro) -> /robot_description and the
    base_footprint -> base_laser static TF
  * spawn_entity -> the robot: planar_move (/cmd_vel in, /odom + odom->base_footprint
    TF out), a 2D lidar (/scan in base_laser)
  * dicerox_mapping/scan_frame_republisher -> /scan_flat (the topic SLAM + Nav2 use)

Then add SLAM (slam_sim.launch.py) and Nav2 (nav2.launch.py), or use
demo.launch.py to bring up everything at once. Drive manually with:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rescue_nav')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg, 'urdf', 'robot_sim.urdf.xacro')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    declared = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg, 'worlds', 'track.world')),
        DeclareLaunchArgument('gui', default_value='true'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items())
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_robocorea',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'robocorea',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'])

    # Reuse dicerox_mapping's republisher so SLAM/Nav2 see /scan_flat (in base_laser)
    # exactly like on hardware. In sim there is no /filtered_odom gate, so disable it.
    scan_flat = Node(
        package='dicerox_mapping', executable='scan_frame_republisher.py',
        name='scan_frame_republisher', output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_flat',
            'output_frame': 'base_laser',
            'require_odom_before_publish': False,
        }])

    return LaunchDescription(declared + [gzserver, gzclient, rsp, spawn, scan_flat])
