import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from jetson_sensors.lis3mdl_node import magnetic_to_message
from jetson_sensors.mlx90640_node import ExponentialFilter, array_to_image
from jetson_sensors.thermal_visualizer_node import (
    bicubic_resize,
    gaussian_blur,
    image_to_array,
    rgb_message,
    thermal_colormap,
)


def test_array_to_image_preserves_temperatures():
    thermal = np.arange(768, dtype=np.float32).reshape(24, 32) / 10.0
    msg = array_to_image(thermal, Time(sec=1, nanosec=2), 'thermal_link')
    assert (msg.height, msg.width, msg.encoding, msg.step) == (24, 32, '32FC1', 128)
    np.testing.assert_array_equal(image_to_array(msg), thermal)


def test_exponential_filter_reduces_frame_step():
    temporal_filter = ExponentialFilter(0.5)
    temporal_filter.update(np.zeros((24, 32), dtype=np.float32))
    result = temporal_filter.update(np.full((24, 32), 10.0, dtype=np.float32))
    np.testing.assert_allclose(result, 5.0)


def test_bicubic_visualization_dimensions_and_range():
    thermal = np.linspace(20.0, 40.0, 768, dtype=np.float32).reshape(24, 32)
    enlarged = bicubic_resize(thermal, 10)
    assert enlarged.shape == (240, 320)
    rgb = thermal_colormap(enlarged, 20.0, 40.0)
    source = Image()
    msg = rgb_message(rgb, source)
    assert (msg.height, msg.width, msg.encoding, msg.step) == (240, 320, 'rgb8', 960)
    assert len(msg.data) == 240 * 320 * 3


def test_gaussian_filter_preserves_constant_image():
    image = np.full((24, 32), 27.5, dtype=np.float32)
    np.testing.assert_allclose(gaussian_blur(image, 0.8), image, atol=1e-5)


def test_magnetometer_message_uses_standard_tesla_units():
    msg = magnetic_to_message((12.0, -34.0, 56.0), Time(sec=3), 'mag_link')
    assert msg.header.frame_id == 'mag_link'
    np.testing.assert_allclose(
        [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z],
        [12e-6, -34e-6, 56e-6],
    )
