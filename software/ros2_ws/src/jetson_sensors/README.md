# jetson_sensors

RoboCorea Jetson-side I2C sensor package. It owns the MLX90640 thermal camera
and LIS3MDL magnetometer, keeping the public topics stable for the GUI:
`/sensors/thermal` and `/sensors/mag`.

## Topics

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/sensors/thermal_raw` | `sensor_msgs/Image` (`32FC1`) | best effort, depth 1 | Unmodified 32x24 Celsius matrix |
| `/sensors/thermal` | `sensor_msgs/Image` (`32FC1`) | best effort, depth 1 | Temporally filtered 32x24 Celsius matrix used by the GUI |
| `/sensors/thermal_status` | `mlx90640_msgs/ThermalStatus` | reliable, depth 10 | Sequence, temperatures, hotspot, rate and errors |
| `/sensors/mag` | `sensor_msgs/MagneticField` | best effort, depth 1 | LIS3MDL magnetic field in standard Tesla units |
| `/sensors/thermal_color` | `sensor_msgs/Image` (`rgb8`) | best effort, depth 1 | Optional local 320x240 bicubic display image |

Both hardware nodes listen to `/sensors/enable_mask`: bit 0 enables the
magnetometer and bit 1 enables thermal acquisition. They start enabled for
standalone bench use; the GUI publishes the mask and can turn them on/off.

## Install and build

Hardware mode requires the CircuitPython sensor drivers:

```bash
python3 -m pip install \
  adafruit-circuitpython-mlx90640 \
  adafruit-circuitpython-lis3mdl \
  adafruit-extended-bus
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mlx90640_msgs jetson_sensors
source install/setup.bash
```

The node user must have access to the configured `/dev/i2c-*` bus. Defaults live
in `config/jetson_sensors.yaml`: MLX90640 at `0x33`, LIS3MDL at `0x1c`, both on
bus `7`.

## Run

Jetson hardware drivers:

```bash
ros2 launch jetson_sensors jetson_sensors.launch.py
```

Dummy sensors:

```bash
ros2 launch jetson_sensors jetson_sensors.launch.py \
  use_dummy_thermal:=true use_dummy_mag:=true
```

Thermal-only local test, including the optional color visualizer:

```bash
ros2 launch jetson_sensors thermal.launch.py use_dummy:=true
rqt_image_view /sensors/thermal_color
```

Run only the receiver-side display processing on the operator computer:

```bash
ros2 launch jetson_sensors visualizer.launch.py
```

## Tuning

Settings are in `config/jetson_sensors.yaml`.

- `refresh_rate` and `publish_rate`: MLX90640 defaults to 8 Hz. Try 16 Hz only
  with a stable, fast I2C bus and short wiring.
- `filter_alpha`: thermal temporal filter strength. `1.0` disables smoothing;
  smaller values are smoother but add lag. `0.55` is a low-latency default.
- `lis3mdl_node.address`: use `28` (`0x1c`) by default, or `30` (`0x1e`) if the
  LIS3MDL board straps SDO/SA1 high.
- `gaussian_sigma`: receiver-side spatial blur in source pixels. It defaults to
  `0.0` because Gaussian blur can hide small hotspots.
- `visualization_scale`: receiver-side bicubic scale. `10` produces 320x240.

Check operation with:

```bash
ros2 topic hz /sensors/thermal --qos-reliability best_effort
ros2 topic echo /sensors/thermal_status
ros2 topic echo /sensors/mag --qos-reliability best_effort
```
