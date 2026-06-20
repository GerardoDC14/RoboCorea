"""SDLS servo + joystick teleop (joy_node + joystick_servo)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_teleop')
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'servo.launch.py')))

    joy = Node(package='joy', executable='joy_node', name='joy_node', output='screen')
    joystick = Node(
        package='arm_teleop',
        executable='joystick_servo',
        name='joystick_servo',
        output='screen',
        parameters=[{'start_mode': 'cart'}],
    )

    return LaunchDescription([servo, joy, joystick])
