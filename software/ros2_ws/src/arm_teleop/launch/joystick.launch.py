"""SDLS servo + joystick teleop (joy_node + joystick_servo).

Standalone by default. When pairing with twin.launch.py (which already starts the
servo), pass use_servo:=false so you don't launch a SECOND servo_node — two
servos both publishing /joint_states make the topic run at 2× rate and the arm
visibly wiggle:

    ros2 launch arm_teleop twin.launch.py
    ros2 launch arm_teleop joystick.launch.py use_servo:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_teleop')
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'servo.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_servo')))

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'autorepeat_rate': ParameterValue(
                LaunchConfiguration('joy_autorepeat_rate'), value_type=float),
        }],
    )
    joystick = Node(
        package='arm_teleop',
        executable='joystick_servo',
        name='joystick_servo',
        output='screen',
        parameters=[{
            'start_mode': 'cart',
            'wrist_speed_scale': ParameterValue(
                LaunchConfiguration('wrist_speed_scale'), value_type=float),
            'wrist_slew_rate': ParameterValue(
                LaunchConfiguration('wrist_slew_rate'), value_type=float),
            'trigger_deadzone': ParameterValue(
                LaunchConfiguration('trigger_deadzone'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_servo', default_value='true',
            description='Start the SDLS servo. Set false when pairing with '
                        'twin.launch.py / bringup, which already runs it.'),
        DeclareLaunchArgument(
            'joy_autorepeat_rate',
            default_value='50.0',
            description='Hz for joy_node to resend held controller state.'),
        DeclareLaunchArgument(
            'wrist_speed_scale',
            default_value='0.8',
            description='Extra scale for joystick pitch/roll commands.'),
        DeclareLaunchArgument(
            'wrist_slew_rate',
            default_value='12.0',
            description='Normalized pitch/roll command slew rate per second.'),
        DeclareLaunchArgument(
            'trigger_deadzone',
            default_value='0.15',
            description='Deadzone for LT/RT pitch axes.'),
        servo, joy, joystick,
    ])
