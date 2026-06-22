import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jetson_sensors'), 'config', 'jetson_sensors.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_dummy_thermal', default_value='false'),
        DeclareLaunchArgument('use_dummy_mag', default_value='false'),
        Node(
            package='jetson_sensors',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[config, {'use_dummy': LaunchConfiguration('use_dummy_thermal')}],
        ),
        Node(
            package='jetson_sensors',
            executable='lis3mdl_node',
            name='lis3mdl_node',
            output='screen',
            parameters=[config, {'use_dummy': LaunchConfiguration('use_dummy_mag')}],
        ),
    ])
