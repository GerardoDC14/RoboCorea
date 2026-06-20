"""Software-only digital-twin test: RViz2 + robot_state_publisher + SDLS servo.

No hardware, no esp32_bridge. The servo publishes /joint_states, which
robot_state_publisher turns into TF for RViz's RobotModel display.

Drive it with the keyboard from a SECOND terminal (it reads raw keys, so it
needs its own terminal — run it directly, no launch):

    ros2 run arm_teleop keyboard_servo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_teleop')

    urdf_path = os.path.join(pkg_share, 'urdf', 'dicerox_arm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    rviz_config = os.path.join(pkg_share, 'rviz', 'twin.rviz')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Reuse servo.launch.py so the twin uses the exact same servo + tuning.
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'servo.launch.py')))

    return LaunchDescription([rsp, rviz, servo])
