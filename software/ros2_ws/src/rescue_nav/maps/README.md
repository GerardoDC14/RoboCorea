# Maps

Saved occupancy grids live here. Build one with `slam.launch.py` + teleop, then:

```bash
ros2 run nav2_map_server map_saver_cli -f src/rescue_nav/maps/track
colcon build --packages-select rescue_nav   # installs track.{yaml,pgm} into share/
```

`localization.launch.py` loads `maps/track.yaml` from the installed `share/` by
default. To use a map without rebuilding, pass an absolute path:
`ros2 launch rescue_nav localization.launch.py map:=/abs/path/track.yaml`.
