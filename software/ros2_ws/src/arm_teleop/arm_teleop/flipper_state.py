#!/usr/bin/env python3
"""
Flipper angle bridge: /encoders/flipper (degrees) -> /joint_states (radians).

The esp32_bridge publishes the four VESC-reported flipper angles as a
``std_msgs/Float32MultiArray`` on ``/encoders/flipper`` in the order
``[fl, fr, rl, rr]`` (degrees). The digital twin (robot_state_publisher + the
GUI ``UrdfViewer``) drives the URDF's flipper joints from ``/joint_states``
(radians), so this node republishes those four values as a ``JointState`` under
the combined URDF's flipper joint names.

It publishes ONLY the flipper joints. The arm joints are published separately by
``servo_node`` on the same ``/joint_states`` topic; both robot_state_publisher
and the GUI viewer merge per-joint, so the two publishers coexist without
fighting (each message only updates the joints it names).

Encoder convention vs. URDF convention (zero offset, direction, 0..360 wrap) is
bench-tunable via the ``angle_offsets_deg``, ``joint_signs`` and ``wrap_pm180``
parameters — see ``config.h``/``flipper_position.lisp`` for the firmware side.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class FlipperState(Node):
    def __init__(self):
        super().__init__('flipper_state')

        # [fl, fr, rl, rr] -> URDF flipper joints (chassis corners). Reorder this
        # if the bench mapping differs from the firmware's encoder order.
        self.declare_parameter(
            'joint_names',
            ['Flipper1J', 'Flipper2J', 'Flipper3J', 'Flipper4J'])
        self.declare_parameter('input_topic', '/encoders/flipper')
        # Per-joint zero offset (deg) applied before the sign/wrap.
        self.declare_parameter('angle_offsets_deg', [0.0, 0.0, 0.0, 0.0])
        # Per-joint direction (+1 / -1) to match the URDF axis sense.
        self.declare_parameter('joint_signs', [1.0, 1.0, 1.0, 1.0])
        # Wrap (value+offset) into [-180, 180] deg so a 0..360 encoder maps onto
        # the revolute joints' signed range. Disable if the encoder is already
        # signed and you want unwrapped continuous rotation.
        self.declare_parameter('wrap_pm180', True)
        # Re-publish the last value at this rate so TF stays fresh between bursts
        # of telemetry. 0 disables the timer (publish only on receipt).
        self.declare_parameter('publish_rate', 20.0)

        self.joint_names = [str(n) for n in
                            self.get_parameter('joint_names').value]
        self.offsets = [float(v) for v in
                        self.get_parameter('angle_offsets_deg').value]
        self.signs = [float(v) for v in
                      self.get_parameter('joint_signs').value]
        self.wrap = bool(self.get_parameter('wrap_pm180').value)
        rate = float(self.get_parameter('publish_rate').value)
        in_topic = str(self.get_parameter('input_topic').value)

        n = len(self.joint_names)
        # Pad/truncate the per-joint tuning to match joint_names length.
        self.offsets = (self.offsets + [0.0] * n)[:n]
        self.signs = (self.signs + [1.0] * n)[:n]

        self._last = None   # last computed positions (rad) or None

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # esp32_bridge publishes /encoders/flipper BEST_EFFORT (sensor stream), so
        # the subscription MUST be best-effort too — a default (reliable) sub is
        # QoS-incompatible and silently receives nothing, freezing the twin's
        # flippers and the body-collision flipper obstacles while the odometry
        # panel (also best-effort) still shows the angles.
        self.sub = self.create_subscription(
            Float32MultiArray, in_topic, self._on_flipper,
            qos_profile_sensor_data)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'flipper_state: {in_topic} (deg) -> /joint_states {self.joint_names} '
            f'(rad), wrap_pm180={self.wrap}, republish={rate:g} Hz')

    def _on_flipper(self, msg: Float32MultiArray):
        data = list(msg.data)
        if not data:
            return
        n = min(len(self.joint_names), len(data))
        pos = []
        for i in range(n):
            deg = self.signs[i] * (float(data[i]) + self.offsets[i])
            if self.wrap:
                deg = (deg + 180.0) % 360.0 - 180.0
            pos.append(math.radians(deg))
        self._last = (self.joint_names[:n], pos)
        self._publish()

    def _on_timer(self):
        if self._last is not None:
            self._publish()

    def _publish(self):
        names, pos = self._last
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = pos
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = FlipperState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
