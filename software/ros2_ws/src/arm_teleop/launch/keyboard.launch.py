"""SDLS servo + keyboard teleop.

Standalone by default. Pass use_servo:=false when pairing with twin.launch.py
(which already runs the servo) to avoid starting a SECOND servo_node — two
servos publishing /joint_states double the topic rate and make the arm wiggle.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_teleop')
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'servo.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_servo')))

    keyboard = Node(
        package='arm_teleop',
        executable='keyboard_servo',
        name='keyboard_servo',
        output='screen',
        emulate_tty=True,
        parameters=[{'start_mode': 'cart'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_servo', default_value='true',
            description='Start the SDLS servo. Set false when pairing with '
                        'twin.launch.py / bringup, which already runs it.'),
        servo, keyboard,
    ])
