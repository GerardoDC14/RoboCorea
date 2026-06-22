import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from jetson_sensors.common import EnableMaskGate, sensor_qos


MICROTESLA_TO_TESLA = 1e-6


class DummyMagReader:
    """Generate a slow magnetic-field sweep for ROS-side testing."""

    def __init__(self) -> None:
        self.phase = 0.0

    def read(self) -> tuple[float, float, float]:
        self.phase += 0.04
        return (
            32.0 + 6.0 * math.sin(self.phase),
            -11.0 + 4.0 * math.cos(self.phase * 0.7),
            18.0 + 2.0 * math.sin(self.phase * 0.31),
        )


class Lis3mdlReader:
    """Read microtesla values from a LIS3MDL over Linux I2C."""

    def __init__(self, bus: int, address: int) -> None:
        try:
            from adafruit_extended_bus import ExtendedI2C as I2C
            import adafruit_lis3mdl
        except ImportError as exc:
            raise RuntimeError(
                'Hardware mode requires adafruit-circuitpython-lis3mdl and '
                'adafruit-extended-bus'
            ) from exc

        self._sensor = adafruit_lis3mdl.LIS3MDL(I2C(bus), address=address)

    def read(self) -> tuple[float, float, float]:
        x, y, z = self._sensor.magnetic
        return float(x), float(y), float(z)


def magnetic_to_message(
    magnetic_ut: tuple[float, float, float],
    stamp,
    frame_id: str,
) -> MagneticField:
    """Convert LIS3MDL microtesla readings to a standard ROS MagneticField."""
    msg = MagneticField()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.magnetic_field.x = float(magnetic_ut[0]) * MICROTESLA_TO_TESLA
    msg.magnetic_field.y = float(magnetic_ut[1]) * MICROTESLA_TO_TESLA
    msg.magnetic_field.z = float(magnetic_ut[2]) * MICROTESLA_TO_TESLA
    return msg


class Lis3mdlNode(Node):
    def __init__(self) -> None:
        super().__init__('lis3mdl_node')
        self.declare_parameter('bus', 7)
        self.declare_parameter('address', 0x1C)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'mag_link')
        self.declare_parameter('topic', '/sensors/mag')
        self.declare_parameter('use_dummy', False)
        self.declare_parameter('enable_mask_topic', '/sensors/enable_mask')
        self.declare_parameter('enable_mask_bit', 0)
        self.declare_parameter('start_enabled', True)

        bus = int(self.get_parameter('bus').value)
        address = int(self.get_parameter('address').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('topic').value)
        use_dummy = bool(self.get_parameter('use_dummy').value)
        enable_mask_topic = str(self.get_parameter('enable_mask_topic').value)
        enable_mask_bit = int(self.get_parameter('enable_mask_bit').value)
        start_enabled = bool(self.get_parameter('start_enabled').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.reader = DummyMagReader() if use_dummy else Lis3mdlReader(bus, address)
        self.publisher = self.create_publisher(MagneticField, topic, sensor_qos())
        self.enable_gate = EnableMaskGate(
            self,
            enable_mask_topic,
            enable_mask_bit,
            start_enabled,
            'LIS3MDL',
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_sample)
        self.total_read_errors = 0
        self.consecutive_read_errors = 0
        self.last_sample_time = None
        mode = 'dummy' if use_dummy else f'I2C bus {bus}, address {address:#x}'
        enabled = 'enabled' if self.enable_gate.enabled else 'disabled'
        self.get_logger().info(
            f'Publishing {topic} at {publish_rate:g} Hz from {mode}; '
            f'currently {enabled}'
        )

    def publish_sample(self) -> None:
        if not self.enable_gate.enabled:
            return
        try:
            magnetic_ut = self.reader.read()
        except (OSError, RuntimeError, ValueError) as exc:
            self.total_read_errors += 1
            self.consecutive_read_errors += 1
            if self.consecutive_read_errors == 1 or self.consecutive_read_errors % 50 == 0:
                self.get_logger().warning(
                    f'Magnetometer read failed '
                    f'({self.consecutive_read_errors} consecutive): {exc}'
                )
            return

        self.last_sample_time = time.monotonic()
        self.consecutive_read_errors = 0
        stamp = self.get_clock().now().to_msg()
        self.publisher.publish(
            magnetic_to_message(magnetic_ut, stamp, self.frame_id)
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Lis3mdlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass


if __name__ == '__main__':
    main()
