import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jetson_sensors'), 'config', 'jetson_sensors.yaml'
    )
    return LaunchDescription([
        Node(
            package='jetson_sensors',
            executable='thermal_visualizer_node',
            name='thermal_visualizer_node',
            output='screen',
            parameters=[config],
        ),
    ])
