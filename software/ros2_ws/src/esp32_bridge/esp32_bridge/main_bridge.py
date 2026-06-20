"""
RoboCorea ESP32 ⇄ ROS 2 bridge (Jetson side)
=============================================
Runs on the Jetson Orin Nano. Owns the USB-serial link to the ESP32, decodes the
binary protocol into ROS 2 topics, and serializes inbound topics back into frames.

Binary frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
              CRC = XOR of TYPE ^ LEN_H ^ LEN_L ^ all payload bytes.
The struct formats below MUST stay byte-identical to firmware include/robot_types.h.

Published (ESP32 → PC)
──────────────────────
/robot/telemetry      std_msgs/Float32MultiArray  [spd_l_rpm, spd_r_rpm, flipper_deg, uptime_s]
/robot/mode           std_msgs/String             INIT/STANDBY/NORMAL/ARM/ESTOP/FLIPPER
/robot/flags          std_msgs/UInt8              bit0 ppm, bit1 sensors, bit2 can, bit3 estop
/robot/ppm            std_msgs/Int16MultiArray    raw PPM µs [ch1..ch6]
/robot/status         diagnostic_msgs/DiagnosticArray
/robot/deadband       std_msgs/Float32            normalised RC deadband (echoed from calib)
/encoders/tracks      geometry_msgs/Vector3       x=left_rpm, y=right_rpm
/encoders/flipper     std_msgs/Float32MultiArray  [fl, fr, rl, rr] degrees
/sensors/imu          sensor_msgs/Imu             BNO055 orientation + accel + gyro
/sensors/mag          sensor_msgs/MagneticField   LIS3MDL XYZ
/motors/vesc_status   std_msgs/Float32MultiArray  [id, erpm, current_A, duty, t_fet, t_mot, v_in]
/motors/odrive_status std_msgs/Float32MultiArray  [joint, pos_turns, vel_turns_s, iq_A, bus_V, bus_A]
/motors/lktech_status std_msgs/Float32MultiArray  [joint, motor_id, temp_C, iq_A, dps, angle, out_deg]
/motors/ze300_status  std_msgs/Float32MultiArray  [id, temp_C, iq_A, rpm, single_turn, pos_counts, out_deg]
/motors/odrive_error  std_msgs/Float32MultiArray  [node_id, motor_error]
/gripper              std_msgs/Float32            normalised gripper command from RC

Subscribed (PC → ESP32)
───────────────────────
/robot/estop          std_msgs/Bool               True = ESTOP, False = ESTOP_CLEAR
/joint_states         sensor_msgs/JointState      arm joint positions (rad) → MSG_ARM_JOINTS
/sensors/enable_mask  std_msgs/UInt8              bit0 mag, bit3 imu (thermal/gas unused here)
/robot/keybind        std_msgs/UInt8MultiArray    15 bytes (3 modes × 5 channels)
/robot/ppm_calib      std_msgs/UInt16MultiArray   19 values (6ch × min/neu/max + deadband)

Parameters
──────────
serial_port (str)  /dev/ttyUSB0
baud_rate   (int)  921600
joint_names (str[]) URDF joint names in J1..J6 order (for /joint_states mapping)
joint_command_signs (float[]) URDF radians → firmware physical-degree signs
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial

from std_msgs.msg import (
    Bool, String, UInt8, UInt16, Float32, Float32MultiArray,
    Int16MultiArray, UInt8MultiArray, UInt16MultiArray,
)
from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger

# ─── Protocol IDs (must match config.h) ──────────────────────────────────────
MSG_TELEMETRY    = 0x01
MSG_MAG          = 0x03
MSG_STATUS       = 0x05
MSG_IMU          = 0x06
MSG_ENCODER_EXT  = 0x07
MSG_VESC_STATUS  = 0x08
MSG_ODRIVE_STATUS = 0x0A
MSG_LKTECH_STATUS = 0x0B
MSG_ZE300_STATUS  = 0x0C
MSG_ODRIVE_ERROR  = 0x0D
MSG_ARM_LIFECYCLE = 0x0E
MSG_GRIPPER       = 0x16

MSG_ARM_JOINTS    = 0x10
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13
MSG_KEYBIND       = 0x14
MSG_PPM_CALIB     = 0x15
MSG_ARM_INIT      = 0x17
MSG_ARM_DISARM    = 0x18

# Arm lifecycle state codes (match firmware ArmState)
ARM_STATE_NAMES = {0: 'UNINIT', 1: 'INITIALIZING', 2: 'READY', 3: 'FAULT'}

MODE_NAMES = {0: 'INIT', 1: 'STANDBY', 2: 'NORMAL', 3: 'ARM', 4: 'ESTOP', 5: 'FLIPPER'}

PPM_CHANNELS = 6
MAX_PAYLOAD_LEN = 2048   # any larger "length" is a false SOF match → resync


def _build_frame(msg_type: int, payload: bytes) -> bytes:
    len_h = (len(payload) >> 8) & 0xFF
    len_l = len(payload) & 0xFF
    crc = msg_type ^ len_h ^ len_l
    for b in payload:
        crc ^= b
    return bytes([0xAA, 0x55, msg_type, len_h, len_l]) + payload + bytes([crc])


class ESP32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('reconnect_period', 3.0)
        # Must match the names the arm servo publishes on /joint_states and the
        # URDF (arm_teleop / dicerox_arm.urdf uses capitalized Joint1..Joint6).
        self.declare_parameter(
            'joint_names',
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        # Match the working Dicerox bridge convention: MoveIt/URDF joint radians
        # are converted to the firmware's physical joint-degree frame before the
        # ESP32 applies its per-motor gear/direction mapping.
        self.declare_parameter('joint_command_signs', [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0])

        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._joint_command_signs = [float(v) for v in self.get_parameter('joint_command_signs').value]
        if len(self._joint_command_signs) != 6:
            self.get_logger().warn(
                f'joint_command_signs must have 6 values, got {len(self._joint_command_signs)}; using defaults')
            self._joint_command_signs = [-1.0, -1.0, -1.0, -1.0, -1.0, 1.0]
        self._ser = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        # Publishers
        self._pub_telemetry = self.create_publisher(Float32MultiArray, '/robot/telemetry', sensor_qos)
        self._pub_mode      = self.create_publisher(String,            '/robot/mode',      sensor_qos)
        self._pub_flags     = self.create_publisher(UInt8,             '/robot/flags',     sensor_qos)
        self._pub_ppm       = self.create_publisher(Int16MultiArray,   '/robot/ppm',       sensor_qos)
        self._pub_status    = self.create_publisher(DiagnosticArray,   '/robot/status',    sensor_qos)
        self._pub_deadband  = self.create_publisher(Float32,           '/robot/deadband',  latched_qos)
        self._pub_tracks    = self.create_publisher(Vector3,           '/encoders/tracks', sensor_qos)
        self._pub_flipper   = self.create_publisher(Float32MultiArray, '/encoders/flipper', sensor_qos)
        self._pub_imu       = self.create_publisher(Imu,               '/sensors/imu',     sensor_qos)
        self._pub_mag       = self.create_publisher(MagneticField,     '/sensors/mag',     sensor_qos)
        self._pub_vesc      = self.create_publisher(Float32MultiArray, '/motors/vesc_status',   sensor_qos)
        self._pub_odrive    = self.create_publisher(Float32MultiArray, '/motors/odrive_status', sensor_qos)
        self._pub_lktech    = self.create_publisher(Float32MultiArray, '/motors/lktech_status', sensor_qos)
        self._pub_ze300     = self.create_publisher(Float32MultiArray, '/motors/ze300_status',  sensor_qos)
        self._pub_odrv_err  = self.create_publisher(Float32MultiArray, '/motors/odrive_error',  sensor_qos)
        self._pub_gripper   = self.create_publisher(Float32,           '/gripper',         sensor_qos)
        # Arm safety lifecycle (latched so a late-joining GUI/servo sees current state)
        self._pub_arm_state = self.create_publisher(String,            '/arm/state',       latched_qos)
        self._pub_arm_fault = self.create_publisher(Bool,              '/arm/fault',       latched_qos)
        self._pub_arm_presence = self.create_publisher(UInt16,          '/arm/can_presence', latched_qos)
        self._last_arm_state = None

        # Arm arm/disarm services (operator → ESP32 explicit lifecycle commands)
        self.create_service(Trigger, '/arm/arm',    self._srv_arm)
        self.create_service(Trigger, '/arm/disarm', self._srv_disarm)

        # Subscribers
        self.create_subscription(Bool,       '/robot/estop',         self._on_estop,         10)
        self.create_subscription(JointState, '/joint_states',        self._on_joint_states,  10)
        self.create_subscription(UInt8,      '/sensors/enable_mask', self._on_sensor_enable, 10)
        self.create_subscription(UInt8MultiArray,  '/robot/keybind',   self._on_keybind,   latched_qos)
        self.create_subscription(UInt16MultiArray, '/robot/ppm_calib', self._on_ppm_calib, latched_qos)

        # Dispatch table
        self._handlers = {
            MSG_TELEMETRY:     self._handle_telemetry,
            MSG_MAG:           self._handle_mag,
            MSG_STATUS:        self._handle_status,
            MSG_IMU:           self._handle_imu,
            MSG_ENCODER_EXT:   self._handle_encoder_ext,
            MSG_VESC_STATUS:   self._handle_vesc_status,
            MSG_ODRIVE_STATUS: self._handle_odrive_status,
            MSG_LKTECH_STATUS: self._handle_lktech_status,
            MSG_ZE300_STATUS:  self._handle_ze300_status,
            MSG_ODRIVE_ERROR:  self._handle_odrive_error,
            MSG_ARM_LIFECYCLE: self._handle_arm_lifecycle,
            MSG_GRIPPER:       self._handle_gripper,
        }

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.get_logger().info('RoboCorea ESP32 bridge ready (serial handled asynchronously)')

    # ── Serial RX ────────────────────────────────────────────────────────────
    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=0.1)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self.get_logger().info(f'Opened {self._serial_port} @ {self._baud_rate}')
            # Start with all sensors off; the GUI enables them via /sensors/enable_mask.
            self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([0x00])))
            return True
        except (serial.SerialException, OSError) as e:
            self._ser = None
            self.get_logger().warn(
                f'{self._serial_port} unavailable ({e}); retry in {self._reconnect_period:.0f}s')
            return False

    def _rx_loop(self):
        state = 'SOF0'
        msg_type = length = running_crc = 0
        payload = bytearray()

        while rclpy.ok():
            if self._ser is None:
                if not self._open_serial():
                    time.sleep(self._reconnect_period)
                    continue
                state, payload, running_crc = 'SOF0', bytearray(), 0

            try:
                raw = self._ser.read(256)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read error: {e}; reconnecting')
                self._close_serial()
                continue
            if not raw:
                continue

            for byte in raw:
                if state == 'SOF0':
                    if byte == 0xAA:
                        state = 'SOF1'
                elif state == 'SOF1':
                    state = 'TYPE' if byte == 0x55 else 'SOF0'
                elif state == 'TYPE':
                    msg_type = byte
                    running_crc = byte
                    state = 'LEN_H'
                elif state == 'LEN_H':
                    length = byte << 8
                    running_crc ^= byte
                    state = 'LEN_L'
                elif state == 'LEN_L':
                    length |= byte
                    running_crc ^= byte
                    payload = bytearray()
                    if length > MAX_PAYLOAD_LEN:
                        state = 'SOF0'
                    else:
                        state = 'CRC' if length == 0 else 'PAYLOAD'
                elif state == 'PAYLOAD':
                    payload.append(byte)
                    running_crc ^= byte
                    if len(payload) >= length:
                        state = 'CRC'
                elif state == 'CRC':
                    if byte == running_crc:
                        self._dispatch(msg_type, bytes(payload))
                    state = 'SOF0'

    def _dispatch(self, msg_type: int, payload: bytes):
        handler = self._handlers.get(msg_type)
        if handler is None:
            return
        try:
            handler(payload)
        except struct.error as e:
            self.get_logger().warn(f'Parse error type=0x{msg_type:02X}: {e}')

    # ── ESP32 → PC handlers ──────────────────────────────────────────────────
    def _handle_telemetry(self, payload: bytes):
        fmt = '<BB' + 'H' * PPM_CHANNELS + 'hhhI'
        if len(payload) < struct.calcsize(fmt):
            return
        fields = struct.unpack_from(fmt, payload)
        mode_val, flags = fields[0], fields[1]
        ppm = list(fields[2:2 + PPM_CHANNELS])
        spd_l_x10, spd_r_x10, flip_x10, uptime_ms = fields[2 + PPM_CHANNELS:]

        m = String(); m.data = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        self._pub_mode.publish(m)

        f = UInt8(); f.data = flags
        self._pub_flags.publish(f)

        p = Int16MultiArray(); p.data = [int(v) for v in ppm]
        self._pub_ppm.publish(p)

        t = Float32MultiArray()
        t.data = [spd_l_x10 / 10.0, spd_r_x10 / 10.0, flip_x10 / 10.0, uptime_ms / 1000.0]
        self._pub_telemetry.publish(t)

        v = Vector3(); v.x = spd_l_x10 / 10.0; v.y = spd_r_x10 / 10.0
        self._pub_tracks.publish(v)

    def _handle_encoder_ext(self, payload: bytes):
        fmt = '<hhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        fl, fr, rl, rr = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [fl / 10.0, fr / 10.0, rl / 10.0, rr / 10.0]
        self._pub_flipper.publish(msg)

    def _handle_imu(self, payload: bytes):
        fmt = '<' + 'h' * 9 + 'B'
        if len(payload) < struct.calcsize(fmt):
            return
        (yaw10, pitch10, roll10, ax100, ay100, az100,
         gx1000, gy1000, gz1000, calib) = struct.unpack_from(fmt, payload)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        yaw, pitch, roll = (math.radians(yaw10 / 10.0),
                            math.radians(pitch10 / 10.0),
                            math.radians(roll10 / 10.0))
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.linear_acceleration.x = ax100 / 100.0
        msg.linear_acceleration.y = ay100 / 100.0
        msg.linear_acceleration.z = az100 / 100.0
        msg.angular_velocity.x = gx1000 / 1000.0
        msg.angular_velocity.y = gy1000 / 1000.0
        msg.angular_velocity.z = gz1000 / 1000.0
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self._pub_imu.publish(msg)

    def _handle_mag(self, payload: bytes):
        fmt = '<hhh'
        if len(payload) < struct.calcsize(fmt):
            return
        x, y, z = struct.unpack_from(fmt, payload)
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mag_link'
        msg.magnetic_field.x = float(x)
        msg.magnetic_field.y = float(y)
        msg.magnetic_field.z = float(z)
        self._pub_mag.publish(msg)

    def _handle_vesc_status(self, payload: bytes):
        fmt = '<Bihhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        vid, erpm, cur10, duty1000, tfet10, tmot10, vin10 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(vid), float(erpm), cur10 / 10.0, duty1000 / 1000.0,
                    tfet10 / 10.0, tmot10 / 10.0, vin10 / 10.0]
        self._pub_vesc.publish(msg)

    def _handle_odrive_status(self, payload: bytes):
        fmt = '<Bhhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint, pos100, vel100, iq100, bv10, bi100 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(joint), pos100 / 100.0, vel100 / 100.0,
                    iq100 / 100.0, bv10 / 10.0, bi100 / 100.0]
        self._pub_odrive.publish(msg)

    def _handle_lktech_status(self, payload: bytes):
        fmt = '<BBbhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint, mid, temp, iq100, dps, angle, out10 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(joint), float(mid), float(temp), iq100 / 100.0,
                    float(dps), float(angle), out10 / 10.0]
        self._pub_lktech.publish(msg)

    def _handle_ze300_status(self, payload: bytes):
        fmt = '<Bbhhhih'
        if len(payload) < struct.calcsize(fmt):
            return
        dev, temp, iq1000, rpm100, st, pos, out10 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(dev), float(temp), iq1000 / 1000.0, rpm100 / 100.0,
                    float(st), float(pos), out10 / 10.0]
        self._pub_ze300.publish(msg)

    def _handle_odrive_error(self, payload: bytes):
        fmt = '<BQ'
        if len(payload) < struct.calcsize(fmt):
            return
        node_id, err = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(node_id), float(err)]
        self._pub_odrv_err.publish(msg)
        if err != 0:
            self.get_logger().warn(f'ODrive node {node_id} error: 0x{err:016X}')

    def _handle_gripper(self, payload: bytes):
        if len(payload) < 2:
            return
        val1000, = struct.unpack_from('<h', payload)
        m = Float32(); m.data = val1000 / 1000.0
        self._pub_gripper.publish(m)

    def _handle_status(self, payload: bytes):
        if len(payload) < 4:
            return
        mode_val, flags, sensor_mask, _ = struct.unpack_from('<BBBB', payload)
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = 'RoboCorea ESP32'
        st.level = DiagnosticStatus.OK
        st.message = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        st.values = [
            KeyValue(key='mode',        value=str(mode_val)),
            KeyValue(key='ppm_ok',      value=str(bool(flags & 0x01))),
            KeyValue(key='sensors_on',  value=str(bool(flags & 0x02))),
            KeyValue(key='can_ok',      value=str(bool(flags & 0x04))),
            KeyValue(key='estop',       value=str(bool(flags & 0x08))),
            KeyValue(key='sensor_mask', value=hex(sensor_mask)),
        ]
        if flags & 0x08:
            st.level = DiagnosticStatus.ERROR
            st.message = 'ESTOP ACTIVE'
        diag.status = [st]
        self._pub_status.publish(diag)

    # ── PC → ESP32 ───────────────────────────────────────────────────────────
    def _send(self, frame: bytes):
        if self._ser is None:
            return
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write error: {e}; reconnecting')
            self._close_serial()

    def _close_serial(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._send(_build_frame(MSG_ESTOP, b''))
            self.get_logger().warn('Sent ESTOP')
        else:
            self._send(_build_frame(MSG_ESTOP_CLEAR, b''))

    # ── Arm safety lifecycle ─────────────────────────────────────────────────
    def _handle_arm_lifecycle(self, payload):
        if len(payload) < 7:
            return
        state, fault_code, can_fail, motor_fail, eflg = struct.unpack('<BBHHB', payload[:7])
        presence = 0
        if len(payload) >= 9:
            presence, = struct.unpack('<H', payload[7:9])
        name = ARM_STATE_NAMES.get(state, f'UNKNOWN({state})')
        if name != self._last_arm_state:
            self._last_arm_state = name
            self.get_logger().info(
                f'arm lifecycle: {name} fault={fault_code} '
                f'can_fail={can_fail} motor_fail={motor_fail} eflg=0x{eflg:02X} '
                f'presence=0x{presence:04X}')
        sm = String(); sm.data = name; self._pub_arm_state.publish(sm)
        fb = Bool(); fb.data = (state == 3); self._pub_arm_fault.publish(fb)
        pm = UInt16(); pm.data = presence; self._pub_arm_presence.publish(pm)

    def _srv_arm(self, request, response):
        self._send(_build_frame(MSG_ARM_INIT, b''))
        response.success = True
        response.message = 'arm init/arm requested'
        self.get_logger().info('Arm: init/arm requested')
        return response

    def _srv_disarm(self, request, response):
        self._send(_build_frame(MSG_ARM_DISARM, b''))
        response.success = True
        response.message = 'arm disarm requested'
        self.get_logger().warn('Arm: disarm requested')
        return response

    def _on_joint_states(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        degs = [
            math.degrees(name_to_pos.get(n, 0.0)) * self._joint_command_signs[i]
            for i, n in enumerate(self._joint_names)
        ]
        payload = struct.pack('<' + 'h' * 6, *[int(d * 100.0) for d in degs])
        self._send(_build_frame(MSG_ARM_JOINTS, payload))

    def _on_sensor_enable(self, msg: UInt8):
        self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([msg.data])))
        self.get_logger().info(f'Sensor enable mask: 0x{msg.data:02X}')

    def _on_keybind(self, msg: UInt8MultiArray):
        if len(msg.data) < 15:
            self.get_logger().warn('keybind needs 15 bytes (3 modes × 5 channels)')
            return
        self._send(_build_frame(MSG_KEYBIND, bytes(msg.data[:15])))
        self.get_logger().info('Keybind table sent to ESP32')

    def _on_ppm_calib(self, msg: UInt16MultiArray):
        if len(msg.data) < 19:
            self.get_logger().warn(f'ppm_calib needs 19 values, got {len(msg.data)}')
            return
        payload = struct.pack('<' + 'HHH' * 6 + 'H', *[int(v) for v in msg.data[:19]])
        self._send(_build_frame(MSG_PPM_CALIB, payload))
        db = Float32(); db.data = msg.data[18] / 1000.0
        self._pub_deadband.publish(db)
        self.get_logger().info(f'PPM calibration sent (deadband={db.data:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
