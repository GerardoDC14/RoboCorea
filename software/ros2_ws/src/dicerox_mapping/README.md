# dicerox_mapping

2D SLAM mapping package for the Dicerox robot. Fuses Slamtec A1 LiDAR scans with ZED2 camera odometry using SLAM Toolbox to build an occupancy-grid map.

## Hardware

- **LiDAR**: Slamtec A1 (range 0.1 – 12.0 m)
- **Camera/Odometry**: ZED2 stereo camera (via `zed-ros2-wrapper`)

## Dependencies

```
slam_toolbox  rclpy  nav_msgs  sensor_msgs  geometry_msgs  tf2_ros  rviz2
```

Install ROS2 dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
colcon build --packages-select dicerox_mapping
source install/setup.bash
```

## Launch

### On the robot (no display)
```bash
ros2 launch dicerox_mapping mapping_robot.launch.py
```

### With RViz visualization (development)
```bash
ros2 launch dicerox_mapping mapping.launch.py
```
Pass `use_rviz:=false` to suppress RViz from the full launch file.

### View map from a remote machine
```bash
ros2 launch dicerox_mapping mapping_viz.launch.py
```

## Common launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_rviz` | `true` | Launch RViz alongside SLAM |
| `zed_odom_topic` | `/zed/zed_node/odom` | ZED odometry input topic |
| `lidar_x/y/z` | `0.0 / 0.0 / 0.05` | LiDAR position relative to `base_footprint` (m) |
| `lidar_yaw` | `3.14159` | LiDAR rotation around Z (rad) |
| `zed_x/y/yaw` | `0.0` | ZED camera offset from robot center |
| `pose_alpha` | `0.35` | X/Y low-pass filter weight (0=frozen, 1=raw) |
| `yaw_alpha` | `0.12` | Yaw low-pass filter weight (0=frozen, 1=raw) |
| `yaw_deadband` | `0.015` | Minimum yaw change to apply (rad) |
| `max_yaw_rate` | `0.6` | Max rotational velocity limit (rad/s) |
| `slam_params_file` | `config/slam_toolbox_params.yaml` | Override SLAM config path |

## Saving a map

While the mapping node is running:
```bash
ros2 launch dicerox_mapping save_map.launch.py
# Optional: override path
ros2 launch dicerox_mapping save_map.launch.py map_name:=$HOME/maps/my_map
```

This saves `my_map.posegraph` and `my_map.data` via the `slam_toolbox/serialize_map` service.

## Localization with a saved map

1. Edit `config/slam_toolbox_localization.yaml` — set `map_file_name` to your saved map path (no extension).
2. Launch:
```bash
ros2 launch dicerox_mapping mapping_robot.launch.py \
  slam_params_file:=$(ros2 pkg prefix dicerox_mapping)/share/dicerox_mapping/config/slam_toolbox_localization.yaml
```

## Recording sensor data

```bash
ros2 launch dicerox_mapping record_bag.launch.py
# Optional: specify output path
ros2 launch dicerox_mapping record_bag.launch.py bag_dir:=$HOME/bags/my_run
```

Recorded topics: `/scan`, `/scan_flat`, `/zed/zed_node/odom`, `/filtered_odom`, `/tf`, `/tf_static`

## TF tree

```
map
 └── odom          ← published by slam_toolbox
      └── base_footprint   ← published by zed_planar_odom
           └── base_laser  ← static transform (lidar_x/y/z/yaw args)
```
