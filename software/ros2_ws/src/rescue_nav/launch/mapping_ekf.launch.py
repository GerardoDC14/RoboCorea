"""
mapping_ekf.launch.py — SLAM (or localization) on top of the FUSED odometry.

This is the drop-in replacement for `dicerox_mapping mapping.launch.py` when you
want the robot_localization EKF (wheel + ZED VIO + ZED IMU) to drive odom
instead of ZED-only. It:

  * includes dicerox_mapping/mapping.launch.py with publish_odom_tf:=false
    (so zed_planar_odom becomes a pure /filtered_odom *source* — no TF), and
  * includes odom_fusion.launch.py (adaptive node + EKF), which becomes the
    sole owner of odom -> base_footprint.

Resulting TF tree (unchanged for everyone downstream):
    map -> odom -> base_footprint -> base_laser
            ^         ^
            |         └─ EKF (fused)        [was: zed_planar_odom, ZED-only]
            └─ slam_toolbox

IMPORTANT: run the ZED driver with publish_tf:=false too (it must not publish
odom->base either). See architecture §18 item 12.

Args of note:
  slam_params_file : config/slam_toolbox_localization.yaml to LOCALIZE against a
                     saved map instead of building one (mapping default).
  use_rviz         : forwarded to mapping.launch.py.
  imu_yaw_sign     : forwarded to the fusion layer (flip if gyro sign inverted).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_share = get_package_share_directory('dicerox_mapping')
    nav_share = get_package_share_directory('rescue_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')

    mapping_launch = os.path.join(mapping_share, 'launch', 'mapping.launch.py')
    fusion_launch = os.path.join(nav_share, 'launch', 'odom_fusion.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(mapping_share, 'config', 'slam_toolbox_params.yaml'),
            description='Use slam_toolbox_localization.yaml to localize against a saved map.'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),

        # Mapping/localization + planar ZED source + scan reframe + slam_toolbox,
        # but WITHOUT the planar node owning the odom TF (the EKF will).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
                'slam_params_file': slam_params_file,
                'zed_odom_topic': zed_odom_topic,
                'publish_odom_tf': 'false',
            }.items(),
        ),

        # The fusion layer: adaptive covariance + EKF (owns odom -> base_footprint).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'wheel_odom_topic': '/odom/wheel',
                'zed_odom_topic': '/filtered_odom',
                'imu_topic': imu_topic,
                'imu_yaw_sign': imu_yaw_sign,
            }.items(),
        ),
    ])
