"""Standalone view of the full robot (arm + chassis + flippers) in RViz.

    ros2 launch arm_description display.launch.py

Starts robot_state_publisher with the combined URDF, a joint_state_publisher_gui
(sliders for every joint, incl. the flippers) and RViz. Use this to eyeball the
merge (arm bolted on the chassis, flippers articulating) with no hardware and no
servo. For the operator stack (GUI + servo + live flipper feed) use
``ros2 launch gui bringup.launch.py`` instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'dicerox_full.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui (sliders) when true.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            condition=UnlessCondition(use_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
