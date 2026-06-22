"""Bring up the SDLS servo with the dicerox kinematics + tuning params.

The servo now loads the **combined** robot URDF (arm + chassis + flippers,
``arm_description/urdf/dicerox_full.urdf``) so it can avoid the body, not just
self-collide. ArmKinematics still only traces the arm chain (base_link -> Link6),
so the extra chassis/flipper joints are ignored for kinematics; they feed the
body-collision checker. Override with ``robot_description_file:=`` (e.g. the
arm-only ``arm_teleop/urdf/dicerox_arm.urdf`` — body collision then disables
itself cleanly).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('robot_description_file').perform(context)
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return [Node(
        package='arm_teleop',
        executable='sdls_servo',
        name='servo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'robot_description': robot_description},
        ],
    )]


def generate_launch_description():
    default_urdf = os.path.join(
        get_package_share_directory('arm_description'), 'urdf', 'dicerox_full.urdf')
    params_file = PathJoinSubstitution(
        [FindPackageShare('arm_teleop'), 'config', 'servo_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='servo tuning params yaml'),
        DeclareLaunchArgument(
            'robot_description_file', default_value=default_urdf,
            description='URDF the servo loads for kinematics + body collision '
                        '(default: the combined arm+chassis+flippers model).'),
        OpaqueFunction(function=_launch_setup),
    ])
