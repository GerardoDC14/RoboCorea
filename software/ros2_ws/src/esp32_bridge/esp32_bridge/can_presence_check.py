#!/usr/bin/env python3
"""Summarize which expected arm CAN devices are visible through esp32_bridge."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String, UInt16


EXPECTED_ODRIVE = {
    0x10: 'J1 ODrive',
    0x11: 'J2 ODrive',
    0x12: 'J3 ODrive',
}
EXPECTED_ODRIVE_BY_INDEX = {
    0: 0x10,
    1: 0x11,
    2: 0x12,
}
EXPECTED_LKTECH = {
    14: 'J5 LKTech',
    15: 'J6 LKTech',
}
EXPECTED_ZE300 = {
    13: 'J4 ZE300',
}


class CanPresenceCheck(Node):
    def __init__(self):
        super().__init__('can_presence_check')
        self.declare_parameter('window_sec', 8.0)
        self.declare_parameter('print_period_sec', 1.0)

        self.window_sec = float(self.get_parameter('window_sec').value)
        self.print_period_sec = float(self.get_parameter('print_period_sec').value)
        self.started = time.monotonic()
        self.last_print = 0.0

        self.odrive_nodes: dict[int, float] = {}
        self.odrive_errors: dict[int, float] = {}
        self.lktech_ids: dict[int, float] = {}
        self.ze300_ids: dict[int, float] = {}
        self.arm_state = 'unknown'
        self.init_presence_mask = 0

        self.create_subscription(Float32MultiArray, '/motors/odrive_status', self._odrive_status_cb, 20)
        self.create_subscription(Float32MultiArray, '/motors/odrive_error', self._odrive_error_cb, 20)
        self.create_subscription(Float32MultiArray, '/motors/lktech_status', self._lktech_cb, 20)
        self.create_subscription(Float32MultiArray, '/motors/ze300_status', self._ze300_cb, 20)
        self.create_subscription(String, '/arm/state', self._arm_state_cb, 10)
        self.create_subscription(UInt16, '/arm/can_presence', self._presence_cb, 10)
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            'Listening for arm CAN telemetry. Run esp32_bridge, call /arm/arm, '
            'then wait for this report.')

    def _now(self) -> float:
        return time.monotonic()

    def _odrive_status_cb(self, msg: Float32MultiArray):
        if not msg.data:
            return
        idx = int(msg.data[0])
        node = EXPECTED_ODRIVE_BY_INDEX.get(idx)
        if node is not None:
            self.odrive_nodes[node] = self._now()

    def _odrive_error_cb(self, msg: Float32MultiArray):
        if not msg.data:
            return
        node = int(msg.data[0])
        self.odrive_nodes[node] = self._now()
        if len(msg.data) > 1 and int(msg.data[1]) != 0:
            self.odrive_errors[node] = msg.data[1]

    def _lktech_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.lktech_ids[int(msg.data[1])] = self._now()

    def _ze300_cb(self, msg: Float32MultiArray):
        if not msg.data:
            return
        self.ze300_ids[int(msg.data[0])] = self._now()

    def _arm_state_cb(self, msg: String):
        self.arm_state = msg.data

    def _presence_cb(self, msg: UInt16):
        self.init_presence_mask = int(msg.data)
        now = self._now()
        if self.init_presence_mask & (1 << 0):
            self.odrive_nodes[0x10] = now
        if self.init_presence_mask & (1 << 1):
            self.odrive_nodes[0x11] = now
        if self.init_presence_mask & (1 << 2):
            self.odrive_nodes[0x12] = now
        if self.init_presence_mask & (1 << 3):
            self.ze300_ids[13] = now
        if self.init_presence_mask & (1 << 4):
            self.lktech_ids[14] = now
        if self.init_presence_mask & (1 << 5):
            self.lktech_ids[15] = now

    def _fresh(self, seen: dict[int, float], key: int) -> bool:
        stamp = seen.get(key)
        return stamp is not None and self._now() - stamp <= self.window_sec

    def _format_group(self, expected: dict[int, str], seen: dict[int, float]) -> tuple[list[str], list[str]]:
        present = []
        missing = []
        for key, label in expected.items():
            item = f'{label} ({key:#04x})'
            if self._fresh(seen, key):
                present.append(item)
            else:
                missing.append(item)
        return present, missing

    def _tick(self):
        now = self._now()
        if now - self.last_print < self.print_period_sec:
            return
        self.last_print = now

        odrv_present, odrv_missing = self._format_group(EXPECTED_ODRIVE, self.odrive_nodes)
        lk_present, lk_missing = self._format_group(EXPECTED_LKTECH, self.lktech_ids)
        ze_present, ze_missing = self._format_group(EXPECTED_ZE300, self.ze300_ids)
        missing = odrv_missing + lk_missing + ze_missing

        print('\n=== CAN presence check ===')
        print(f'arm_state: {self.arm_state}  init_presence_mask=0x{self.init_presence_mask:04x}')
        print('present:')
        for item in odrv_present + lk_present + ze_present:
            print(f'  OK  {item}')
        print('missing:')
        if missing:
            for item in missing:
                print(f'  --  {item}')
        else:
            print('  none')
        if self.odrive_errors:
            print('odrive_errors:')
            for node, err in sorted(self.odrive_errors.items()):
                print(f'  node {node:#04x}: 0x{int(err):016x}')
        print('Tip: ODrive presence can be detected from /motors/odrive_error even before READY.')


def main(args=None):
    rclpy.init(args=args)
    node = CanPresenceCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
