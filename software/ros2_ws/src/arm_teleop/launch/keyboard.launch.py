"""SDLS servo + keyboard teleop."""

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

    keyboard = Node(
        package='arm_teleop',
        executable='keyboard_servo',
        name='keyboard_servo',
        output='screen',
        emulate_tty=True,
        parameters=[{'start_mode': 'cart'}],
    )

    return LaunchDescription([servo, keyboard])
