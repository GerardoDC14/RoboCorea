"""Workstation operator bring-up: GUI + arm control + digital twin, one launch.

This is the single "run everything on the workstation" entry point. It lives in
the **gui** package (the operator console owns the bring-up). It starts:

  * robot_state_publisher with the COMBINED URDF (arm + chassis + 4 flippers) ->
    /robot_description (the GUI digital twin + RViz read this) and TF.
  * the SDLS servo (arm_teleop/servo.launch.py) -> arm joints on /joint_states,
    with self- AND body-collision avoidance (chassis + flippers, posed from
    /encoders/flipper).
  * flipper_state (arm_teleop) -> the 4 flipper joints on /joint_states from
    /encoders/flipper, so the twin's flippers track the real robot.
  * the operator GUI (gui) — video, dashboard (incl. the arm arm/disarm + mode
    controls), odometry and the digital twin.
  * optionally joystick teleop (joy_node + joystick_servo) and/or RViz.

It does NOT start the esp32_bridge (that runs on the Jetson). On the workstation
/encoders/flipper, /arm/state, /robot/* etc. arrive over DDS from the robot. Run
the bridge separately for hardware; with no bridge the flippers stay level and
the arm still jogs/plans (it just has no live body to avoid beyond the chassis).

Examples:
    ros2 launch gui bringup.launch.py                 # GUI + twin + servo
    ros2 launch gui bringup.launch.py joystick:=true  # + gamepad teleop
    ros2 launch gui bringup.launch.py use_gui:=false use_rviz:=true
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
    arm_teleop_share = get_package_share_directory('arm_teleop')
    arm_desc_share = get_package_share_directory('arm_description')

    urdf_path = os.path.join(arm_desc_share, 'urdf', 'dicerox_full.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    rviz_config = os.path.join(arm_teleop_share, 'rviz', 'twin.rviz')

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_servo = LaunchConfiguration('use_servo')
    joystick = LaunchConfiguration('joystick')

    # /robot_description + TF for the digital twin (GUI) and RViz.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Arm servo (self- + body-collision). Defaults to the combined URDF too.
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_teleop_share, 'launch', 'servo.launch.py')),
        condition=IfCondition(use_servo))

    # /encoders/flipper (deg) -> flipper joints on /joint_states (rad).
    flipper = Node(
        package='arm_teleop',
        executable='flipper_state',
        name='flipper_state',
        output='screen',
    )

    gui = Node(
        package='gui',
        executable='gui',
        name='gui_node',
        output='screen',
        condition=IfCondition(use_gui),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    # Optional gamepad teleop. use_servo:=false here because we already start the
    # servo above — two servo_nodes would double /joint_states and wiggle the arm.
    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_teleop_share, 'launch', 'joystick.launch.py')),
        launch_arguments={'use_servo': 'false'}.items(),
        condition=IfCondition(joystick))

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Start the operator GUI.'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz with the twin config.'),
        DeclareLaunchArgument('use_servo', default_value='true',
                              description='Start the SDLS arm servo.'),
        DeclareLaunchArgument('joystick', default_value='true',
                              description='Start joy_node + joystick_servo teleop.'),
        rsp, servo, flipper, gui, rviz, joy_teleop,
    ])
