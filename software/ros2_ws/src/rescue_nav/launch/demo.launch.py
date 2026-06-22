"""One-shot SIM demo: Gazebo + robot + online SLAM + Nav2 + RViz.

The quickest way to watch the autonomy PoC drive without hardware:

  ros2 launch rescue_nav demo.launch.py
  # then in another terminal, send the robot to the end of the track and back:
  ros2 run rescue_nav waypoint_runner

Online SLAM (slam_sim) builds the map live while Nav2 navigates it, so there is
no map save/reload step in this demo. For the map-first workflow (build + save +
slam_toolbox localization), see the rescue_nav README.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rescue_nav')
    dicerox = get_package_share_directory('dicerox_mapping')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # Pass file paths explicitly: an included launch's DeclareLaunchArgument default
    # is NOT applied when the include is deferred inside a TimerAction, so relying on
    # the defaults here leaves Nav2's params_file empty (open('') -> FileNotFound).
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(dicerox, 'config', 'slam_toolbox_params.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui}.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam_sim.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'slam_params_file': slam_params}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'params_file': nav2_params}.items())

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz), output='screen')

    # Give Gazebo a few seconds to come up before SLAM/Nav2 start consuming TF.
    delayed = TimerAction(period=5.0, actions=[slam, nav2, rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        delayed,
    ])
