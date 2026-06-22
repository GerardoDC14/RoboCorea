import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('dicerox_mapping')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_x     = LaunchConfiguration('lidar_x')
    lidar_y     = LaunchConfiguration('lidar_y')
    lidar_z     = LaunchConfiguration('lidar_z')
    lidar_roll  = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw   = LaunchConfiguration('lidar_yaw')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    zed_x = LaunchConfiguration('zed_x')
    zed_y = LaunchConfiguration('zed_y')
    zed_yaw = LaunchConfiguration('zed_yaw')
    pose_alpha = LaunchConfiguration('pose_alpha')
    yaw_alpha = LaunchConfiguration('yaw_alpha')
    yaw_deadband = LaunchConfiguration('yaw_deadband')
    max_yaw_rate = LaunchConfiguration('max_yaw_rate')
    tf_time_offset = LaunchConfiguration('tf_time_offset')
    input_scan_topic = LaunchConfiguration('input_scan_topic')
    slam_scan_topic = LaunchConfiguration('slam_scan_topic')
    laser_frame = LaunchConfiguration('laser_frame')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_x',     default_value='0.0'),
        DeclareLaunchArgument('lidar_y',     default_value='0.0'),
        DeclareLaunchArgument('lidar_z',     default_value='0.05'),
        DeclareLaunchArgument('lidar_roll',  default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw',   default_value='3.14159'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('zed_x',       default_value='0.0'),
        DeclareLaunchArgument('zed_y',       default_value='0.0'),
        DeclareLaunchArgument('zed_yaw',     default_value='0.0'),
        DeclareLaunchArgument('pose_alpha',  default_value='0.35'),
        DeclareLaunchArgument('yaw_alpha',   default_value='0.12'),
        DeclareLaunchArgument('yaw_deadband', default_value='0.015'),
        DeclareLaunchArgument('max_yaw_rate', default_value='0.6'),
        DeclareLaunchArgument('tf_time_offset', default_value='0.0'),
        DeclareLaunchArgument('input_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan_flat'),
        DeclareLaunchArgument('laser_frame', default_value='base_laser'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='dicerox_mapping',
            executable='zed_planar_odom_node.py',
            name='zed_planar_odom',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_odom_topic': zed_odom_topic,
                'output_odom_topic': '/filtered_odom',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'zed_x': ParameterValue(zed_x, value_type=float),
                'zed_y': ParameterValue(zed_y, value_type=float),
                'zed_yaw': ParameterValue(zed_yaw, value_type=float),
                'pose_alpha': ParameterValue(pose_alpha, value_type=float),
                'yaw_alpha': ParameterValue(yaw_alpha, value_type=float),
                'yaw_deadband': ParameterValue(yaw_deadband, value_type=float),
                'max_yaw_rate': ParameterValue(max_yaw_rate, value_type=float),
                'tf_time_offset': ParameterValue(tf_time_offset, value_type=float),
                'publish_tf': True,
            }],
        ),

        Node(
            package='dicerox_mapping',
            executable='scan_frame_republisher.py',
            name='scan_frame_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_scan_topic': input_scan_topic,
                'output_scan_topic': slam_scan_topic,
                'output_frame': laser_frame,
                'filtered_odom_topic': '/filtered_odom',
                'require_odom_before_publish': True,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=[
                '--x', lidar_x,
                '--y', lidar_y,
                '--z', lidar_z,
                '--roll', lidar_roll,
                '--pitch', lidar_pitch,
                '--yaw', lidar_yaw,
                '--frame-id', 'base_footprint',
                '--child-frame-id', laser_frame,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'scan_topic': slam_scan_topic,
                },
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'mapping.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
