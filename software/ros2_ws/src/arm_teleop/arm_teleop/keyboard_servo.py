#!/usr/bin/env python3
"""
Keyboard teleop for the SDLS servo — momentary (hold-to-move).

Motion happens ONLY while you hold a key; release and it stops. This relies on
the terminal's key auto-repeat: each repeat refreshes a hold deadline, and when
the repeats stop (key released) the command is zeroed after `hold_timeout`.

> Note: most terminals have an initial repeat *delay* (~0.25–0.5 s) before
> repeats begin, so a very brief tap may produce a tiny hitch. For the smoothest
> feel, lower the OS key-repeat delay (e.g. `xset r rate 200 30`) or just hold.
> The joystick path gives true analog proportional control without this caveat.

Publishes TwistStamped on `/servo_node/delta_twist_cmds` and JointJog on
`/servo_node/delta_joint_cmds`.
"""

import sys
import time
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped

DEFAULT_JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
DEFAULT_FRAME_ID = 'base_link'
DEFAULT_PUBLISH_HZ = 50.0

# Cartesian: key -> (vx, vy, vz, wx, wy, wz)
CART_BINDINGS = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    'r': (0, 0, 1, 0, 0, 0),
    'f': (0, 0, -1, 0, 0, 0),
    'u': (0, 0, 0, 1, 0, 0),
    'j': (0, 0, 0, -1, 0, 0),
    'i': (0, 0, 0, 0, 1, 0),
    'k': (0, 0, 0, 0, -1, 0),
    'o': (0, 0, 0, 0, 0, 1),
    'l': (0, 0, 0, 0, 0, -1),
}
JOINT_BINDINGS = {'w': 1.0, 's': -1.0}

SPEED_STEP = 0.1
SPEED_MIN = 0.1
SPEED_MAX = 1.0
MAX_KEY_SELECTABLE_JOINTS = 9


def get_key(settings, timeout=0.02):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(mode, joint_name, n_joints, speed):
    sys.stdout.write('\r\033[K')
    if mode == 'joint':
        sys.stdout.write(
            f'[JOINT] {joint_name}  Speed:{speed:.1f}  '
            f'(hold w/s=jog, 1-{n_joints}=select, m=cart, +/- speed, q=quit)')
    else:
        sys.stdout.write(
            f'[CART]  Speed:{speed:.1f}  '
            f'(hold w/s=X a/d=Y r/f=Z u/j=Roll i/k=Pitch o/l=Yaw, m=joint, q=quit)')
    sys.stdout.flush()


class KeyboardServo(Node):
    def __init__(self):
        super().__init__('keyboard_servo')

        self.declare_parameter('robot_name', 'Dicerox Arm')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('planning_frame', DEFAULT_FRAME_ID)
        self.declare_parameter('twist_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('joint_topic', '/servo_node/delta_joint_cmds')
        self.declare_parameter('start_mode', 'cart')
        self.declare_parameter('speed_scale', 0.5)
        self.declare_parameter('publish_hz', DEFAULT_PUBLISH_HZ)
        # How long after the last key-repeat to keep moving before stopping.
        # Set above your terminal's auto-repeat interval; below its repeat delay
        # is impossible, hence the brief-tap caveat in the module docstring.
        self.declare_parameter('hold_timeout', 0.4)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.joint_names = [str(n) for n in self.get_parameter('joint_names').value]
        self.frame_id = str(self.get_parameter('planning_frame').value)
        self.mode = str(self.get_parameter('start_mode').value).lower()
        self.speed = float(self.get_parameter('speed_scale').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)
        self._joint_select_keys = [str(i + 1) for i in range(len(self.joint_names))]

        if not self.joint_names:
            raise ValueError('joint_names must contain at least one joint.')
        if len(self.joint_names) > MAX_KEY_SELECTABLE_JOINTS:
            raise ValueError(
                f'keyboard_servo supports at most {MAX_KEY_SELECTABLE_JOINTS} joints.')
        if self.mode not in {'cart', 'joint'}:
            raise ValueError("start_mode must be 'cart' or 'joint'.")
        if not SPEED_MIN <= self.speed <= SPEED_MAX:
            raise ValueError(f'speed_scale must be within [{SPEED_MIN}, {SPEED_MAX}].')
        if self.publish_hz <= 0.0:
            raise ValueError('publish_hz must be > 0.0.')

        self.twist_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter('twist_topic').value), 10)
        self.joint_pub = self.create_publisher(
            JointJog, str(self.get_parameter('joint_topic').value), 10)

        self.active_joint = 0
        self._lock = threading.Lock()
        self._cart_cmd = None          # (vx..wz) or None
        self._joint_cmd = None         # (joint_idx, direction) or None
        self._last_key_time = 0.0      # monotonic time of last motion key-repeat

        self.create_timer(1.0 / self.publish_hz, self._publish_cb)

    def _publish_cb(self):
        with self._lock:
            cart = self._cart_cmd
            joint = self._joint_cmd
            held = (time.monotonic() - self._last_key_time) <= self.hold_timeout
            if not held:                # key released → stop
                self._cart_cmd = None
                self._joint_cmd = None
        now = self.get_clock().now().to_msg()

        if held and cart is not None:
            msg = TwistStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.twist.linear.x = float(cart[0]) * self.speed
            msg.twist.linear.y = float(cart[1]) * self.speed
            msg.twist.linear.z = float(cart[2]) * self.speed
            msg.twist.angular.x = float(cart[3]) * self.speed
            msg.twist.angular.y = float(cart[4]) * self.speed
            msg.twist.angular.z = float(cart[5]) * self.speed
            self.twist_pub.publish(msg)
        elif held and joint is not None:
            idx, direction = joint
            msg = JointJog()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.joint_names = [self.joint_names[idx]]
            msg.velocities = [direction * self.speed]
            msg.duration = 0.0
            self.joint_pub.publish(msg)
        else:
            # explicit stop: a zero twist halts the servo promptly on release
            msg = TwistStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            self.twist_pub.publish(msg)

    def handle_key(self, key):
        if key in ('q', '\x03'):
            return False
        if key == 'm':
            self.mode = 'cart' if self.mode == 'joint' else 'joint'
            self._stop()
            return True
        if key in (' ', 'x'):
            self._stop()
            return True
        if key in ('+', '='):
            self.speed = min(SPEED_MAX, round(self.speed + SPEED_STEP, 1))
            return True
        if key == '-':
            self.speed = max(SPEED_MIN, round(self.speed - SPEED_STEP, 1))
            return True

        if self.mode == 'joint':
            if key in self._joint_select_keys:
                self.active_joint = int(key) - 1
                self._stop()
                return True
            if key in JOINT_BINDINGS:
                with self._lock:
                    self._joint_cmd = (self.active_joint, JOINT_BINDINGS[key])
                    self._cart_cmd = None
                    self._last_key_time = time.monotonic()
                return True
        else:
            if key in CART_BINDINGS:
                with self._lock:
                    self._cart_cmd = CART_BINDINGS[key]
                    self._joint_cmd = None
                    self._last_key_time = time.monotonic()
                return True
        return True

    def _stop(self):
        with self._lock:
            self._cart_cmd = None
            self._joint_cmd = None
            self._last_key_time = 0.0


def main():
    rclpy.init()
    node = KeyboardServo()
    settings = termios.tcgetattr(sys.stdin)

    print('\n' + '=' * 64)
    print(f'  {node.robot_name} — Keyboard Servo  (HOLD to move, release to stop)')
    print('=' * 64)
    print('  CART  mode -> hold  w/s=X  a/d=Y  r/f=Z  u/j=Roll  i/k=Pitch  o/l=Yaw')
    print(f'  JOINT mode -> 1-{len(node.joint_names)} select joint, hold w/s to jog')
    print('  m=toggle mode   +/-=speed   space/x=stop   q=quit')
    print('=' * 64 + '\n')

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        print_status(node.mode, node.joint_names[node.active_joint],
                     len(node.joint_names), node.speed)
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                continue
            if not node.handle_key(key):
                break
            print_status(node.mode, node.joint_names[node.active_joint],
                         len(node.joint_names), node.speed)
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f'Error: {e}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nShutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
