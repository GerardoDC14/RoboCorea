"""Bring up the SDLS servo with the dicerox arm kinematics + tuning params."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_teleop')

    # The dicerox arm is plain URDF (no xacro macros); read it directly.
    urdf_path = os.path.join(pkg_share, 'urdf', 'dicerox_arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    params_file = PathJoinSubstitution(
        [FindPackageShare('arm_teleop'), 'config', 'servo_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='servo tuning params yaml'),
        Node(
            package='arm_teleop',
            executable='sdls_servo',
            name='servo_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'robot_description': robot_description},
            ],
        ),
    ])
