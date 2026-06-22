"""slam_toolbox online mapping in simulation, reusing dicerox_mapping's config.

The TF tree in sim (odom->base_footprint from planar_move, base_footprint->base_laser
from the URDF) is identical to hardware, so dicerox_mapping's slam_toolbox_params.yaml
applies unchanged here. Drive with teleop to build the map; save it with
  ros2 run slam_toolbox map_saver_cli -t /map -f <path>      (occupancy grid)
or dicerox_mapping's save_map.launch.py (slam_toolbox serialize).

On HARDWARE you don't use this file — run dicerox_mapping's mapping.launch.py
(ZED + lidar) instead. This is the sim equivalent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dicerox = get_package_share_directory('dicerox_mapping')
    default_params = os.path.join(dicerox, 'config', 'slam_toolbox_params.yaml')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time, 'scan_topic': '/scan_flat'},
            ],
        ),
    ])
