"""Software-only digital-twin test: RViz2 + robot_state_publisher + SDLS servo.

No hardware, no esp32_bridge. robot_state_publisher serves the **combined** URDF
(arm + chassis + flippers); the servo publishes the arm joints on /joint_states,
which RViz's RobotModel renders. The flippers sit at 0 here (no /encoders/flipper
feed) — for the full operator stack incl. the GUI + live flippers use
``ros2 launch gui bringup.launch.py``.

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

    urdf_path = os.path.join(
        get_package_share_directory('arm_description'), 'urdf', 'dicerox_full.urdf')
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

    # Reuse servo.launch.py so the twin uses the exact same servo + tuning
    # (it also defaults to the combined URDF now).
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'servo.launch.py')))

    return LaunchDescription([rsp, rviz, servo])
