from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial device for the ESP32'),
        DeclareLaunchArgument(
            'baud_rate', default_value='921600',
            description='UART baud rate (must match MINIPC_BAUD in config.h)'),
        Node(
            package='esp32_bridge',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate':   LaunchConfiguration('baud_rate'),
            }],
        ),
    ])
