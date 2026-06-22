import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_dir = LaunchConfiguration('bag_dir')

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_bag = os.path.join(os.path.expanduser('~'), 'bags', f'run_{timestamp}')

    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value=default_bag),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_dir,
                '/scan',
                '/scan_flat',
                '/zed/zed_node/odom',
                '/filtered_odom',
                '/tf',
                '/tf_static',
            ],
            output='screen',
        ),
    ])
