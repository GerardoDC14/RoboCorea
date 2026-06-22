#!/usr/bin/env python3
"""
adaptive_odom_covariance.py — dynamic-weight front-end for the odometry EKF.

WHY THIS EXISTS
---------------
``robot_localization``'s EKF already does "dynamic weighting": its Kalman gain
weights every measurement inversely to the covariance carried on that message.
So instead of hand-rolling a filter with hand-tuned weights, we modulate the
*covariance we publish* on each odometry source as a function of live driving
conditions, and let the stock EKF rebalance itself update-to-update.

This node sits between the raw sources and the EKF:

    /odom/wheel            (bridge, VESC tachometers, BEST_EFFORT)
    /filtered_odom         (zed_planar_odom, planar ZED VIO)          ┐
    /zed/.../imu/data      (ZED2 IMU, gyro + orientation)            ─┤
                                                                      ▼
                                  adaptive_odom_covariance
                                                                      ▼
    /odom/wheel_adaptive   vx-only, covariance inflated on slip/climb
    /odom/zed_adaptive     vx/vy/vyaw, covariance inflated on blur/low-confidence
    /imu/adaptive          yaw-rate, reframed to base, lightly inflated on vibration
    /diagnostics/odom_fusion  [slip, tilt, turn_rate, f_wheel, f_zed]  (for rqt_plot)

It also fixes two real-world plumbing issues for free:
  * QoS bridge — /odom/wheel is BEST_EFFORT; the EKF subscribes RELIABLE. We
    subscribe BEST_EFFORT and republish RELIABLE so the EKF actually sees it.
  * Frame normalization — the bridge stamps wheel odom child_frame=base_link
    while the SLAM stack is base_footprint; we restamp to a single base frame,
    and relabel the IMU into that base frame too (see IMU MOUNT note below).

THE COMPLEMENTARY SPLIT (what makes it beat ZED-only)
-----------------------------------------------------
The two sensors fail in opposite regimes, so the EKF config (ekf.yaml) fuses:
  * forward velocity (vx)  : wheel (metrically exact straight-line) + ZED
  * yaw rate (vyaw)        : ZED IMU gyro (immune to track slip) + ZED VIO
  * lateral (vy)           : ZED only
Wheel yaw is never fused (skid-steer slip makes it garbage). This node then
*detunes* each source when its failure mode is active.

ADAPTIVE SIGNALS
----------------
  slip      = |omega_wheel - omega_imu|   track slip / skid turn  -> distrust wheel vx
  tilt      = max(|roll|, |pitch|)         climbing ramp/obstacle  -> distrust wheel vx
  turn_rate = |omega_imu|                  motion blur proxy       -> distrust ZED VIO
  zed_cov   = trace(ZED pose cov)          VIO tracking confidence -> distrust ZED VIO

IMU MOUNT ASSUMPTION
--------------------
We relabel the IMU into ``base_frame`` (identity transform) rather than rely on
a base->zed_imu_link TF, which avoids TF-tree conflicts with the ZED wrapper's
own static frames. This assumes the ZED is mounted **level and forward-facing**
so the IMU's z-gyro == base yaw-rate. If yours is tilted/rotated, set
``imu_yaw_sign`` (to flip) and re-verify the sign on the bench (rotate the robot
CCW -> /imu/adaptive angular_velocity.z must be positive).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def roll_pitch_from_quaternion(q):
    """Return (roll, pitch) in radians from a geometry_msgs Quaternion."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))
    return roll, pitch


class AdaptiveOdomCovariance(Node):

    def __init__(self):
        super().__init__('adaptive_odom_covariance')

        # ── Topics ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_odom_topic', '/odom/wheel')
        self.declare_parameter('zed_odom_topic', '/filtered_odom')
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data')
        self.declare_parameter('wheel_out_topic', '/odom/wheel_adaptive')
        self.declare_parameter('zed_out_topic', '/odom/zed_adaptive')
        self.declare_parameter('imu_out_topic', '/imu/adaptive')
        self.declare_parameter('base_frame', 'base_footprint')

        # ── Base (best-case) measurement variances ────────────────────────
        # These are the variances published when conditions are ideal. The EKF
        # reads them off each message; the factors below scale them up.
        self.declare_parameter('wheel_vx_var', 0.02)     # (m/s)^2  trust straight-line
        self.declare_parameter('zed_vx_var', 0.05)       # (m/s)^2
        self.declare_parameter('zed_vy_var', 0.05)       # (m/s)^2
        self.declare_parameter('zed_vyaw_var', 0.05)     # (rad/s)^2
        self.declare_parameter('imu_vyaw_var', 0.005)    # (rad/s)^2  gyro is good

        # ── Adaptive gains (how hard each signal detunes a source) ─────────
        self.declare_parameter('k_slip', 40.0)           # per (rad/s) of slip
        self.declare_parameter('k_tilt', 30.0)           # per rad of tilt above deadband
        self.declare_parameter('tilt_deadband', 0.10)    # rad (~6 deg) ignore normal pitch
        self.declare_parameter('k_blur', 4.0)            # per (rad/s) of turn rate
        self.declare_parameter('k_zedcov', 0.0)          # per unit ZED pose-cov trace excess
        self.declare_parameter('zedcov_nominal', 0.05)   # expected ZED pose-cov trace
        self.declare_parameter('k_vib', 0.0)             # per (m/s^2) of vibration
        self.declare_parameter('max_factor', 500.0)      # cap on any inflation

        # ── Signal smoothing + IMU sign ───────────────────────────────────
        self.declare_parameter('signal_alpha', 0.3)      # EMA on slip/tilt/turn
        self.declare_parameter('imu_yaw_sign', 1.0)      # flip if mount inverts z-gyro

        g = lambda n: self.get_parameter(n).value
        self.wheel_topic = g('wheel_odom_topic')
        self.zed_topic = g('zed_odom_topic')
        self.imu_topic = g('imu_topic')
        self.base_frame = g('base_frame')

        self.wheel_vx_var = float(g('wheel_vx_var'))
        self.zed_vx_var = float(g('zed_vx_var'))
        self.zed_vy_var = float(g('zed_vy_var'))
        self.zed_vyaw_var = float(g('zed_vyaw_var'))
        self.imu_vyaw_var = float(g('imu_vyaw_var'))

        self.k_slip = float(g('k_slip'))
        self.k_tilt = float(g('k_tilt'))
        self.tilt_deadband = float(g('tilt_deadband'))
        self.k_blur = float(g('k_blur'))
        self.k_zedcov = float(g('k_zedcov'))
        self.zedcov_nominal = float(g('zedcov_nominal'))
        self.k_vib = float(g('k_vib'))
        self.max_factor = float(g('max_factor'))

        self.alpha = clamp(float(g('signal_alpha')), 0.01, 1.0)
        self.imu_yaw_sign = float(g('imu_yaw_sign'))

        # ── Live signal state (EMA-smoothed) ──────────────────────────────
        self.omega_imu = 0.0       # rad/s, base-frame yaw rate from gyro
        self.omega_wheel = 0.0     # rad/s, from wheel twist
        self.tilt = 0.0            # rad
        self.vibration = 0.0       # m/s^2 (|accel| - g), abs
        self.slip = 0.0            # rad/s, smoothed
        self.turn_rate = 0.0       # rad/s, smoothed |omega_imu|
        self.f_wheel = 1.0
        self.f_zed = 1.0
        self._have_imu = False

        # ── QoS ───────────────────────────────────────────────────────────
        # Inputs match their producers; outputs are RELIABLE so the EKF (which
        # subscribes RELIABLE) actually receives them.
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE, depth=10)
        out_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_wheel = self.create_publisher(Odometry, g('wheel_out_topic'), out_qos)
        self.pub_zed = self.create_publisher(Odometry, g('zed_out_topic'), out_qos)
        self.pub_imu = self.create_publisher(Imu, g('imu_out_topic'), out_qos)
        self.pub_diag = self.create_publisher(Float32MultiArray, '/diagnostics/odom_fusion', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        # IMU first so slip/tilt are available when wheel/zed arrive.
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.wheel_topic, self._on_wheel, best_effort)
        self.create_subscription(Odometry, self.zed_topic, self._on_zed, out_qos)

        self.get_logger().info(
            'adaptive_odom_covariance up.\n'
            f'  wheel  {self.wheel_topic} -> {g("wheel_out_topic")} (vx)\n'
            f'  zed    {self.zed_topic} -> {g("zed_out_topic")} (vx,vy,vyaw)\n'
            f'  imu    {self.imu_topic} -> {g("imu_out_topic")} (vyaw, base={self.base_frame})\n'
            f'  signals: slip(k={self.k_slip}) tilt(k={self.k_tilt}) '
            f'blur(k={self.k_blur}) | base_link={self.base_frame}\n'
            '  NOTE: assumes ZED mounted level & forward; verify imu_yaw_sign on bench.'
        )

    # ── IMU: update yaw-rate, tilt, vibration; republish as base-frame yaw-rate
    def _on_imu(self, msg: Imu):
        self._have_imu = True
        wz = self.imu_yaw_sign * msg.angular_velocity.z
        self.omega_imu = wz
        self.turn_rate += self.alpha * (abs(wz) - self.turn_rate)

        roll, pitch = roll_pitch_from_quaternion(msg.orientation)
        tilt = max(abs(roll), abs(pitch))
        self.tilt += self.alpha * (tilt - self.tilt)

        # |accel| minus gravity as a crude vibration proxy
        a = msg.linear_acceleration
        amag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.vibration += self.alpha * (abs(amag - 9.80665) - self.vibration)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame            # relabel into base (see module docstring)
        out.angular_velocity.z = wz
        # Only yaw-rate is fused; mark everything else "unused" with large variance.
        big = 1e6
        out.orientation_covariance = [big, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, big]
        out.angular_velocity_covariance = [big, 0.0, 0.0,
                                           0.0, big, 0.0,
                                           0.0, 0.0, self._imu_vyaw_variance()]
        out.linear_acceleration_covariance = [big, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, big]
        self.pub_imu.publish(out)

    def _imu_vyaw_variance(self):
        f_imu = clamp(1.0 + self.k_vib * self.vibration, 1.0, self.max_factor)
        return self.imu_vyaw_var * f_imu

    # ── Wheel: vx only, inflated on slip + climb ──────────────────────────
    def _on_wheel(self, msg: Odometry):
        self.omega_wheel = msg.twist.twist.angular.z
        slip_inst = abs(self.omega_wheel - self.omega_imu) if self._have_imu else 0.0
        self.slip += self.alpha * (slip_inst - self.slip)

        tilt_excess = max(0.0, self.tilt - self.tilt_deadband)
        self.f_wheel = clamp(1.0 + self.k_slip * self.slip + self.k_tilt * tilt_excess,
                             1.0, self.max_factor)

        out = self._copy_odom(msg)
        out.child_frame_id = self.base_frame             # normalize base_link -> base_footprint
        vx_var = self.wheel_vx_var * self.f_wheel
        self._set_twist_var(out, vx=vx_var)
        self.pub_wheel.publish(out)
        self._publish_diag()

    # ── ZED VIO: vx/vy/vyaw, inflated on blur + low confidence ────────────
    def _on_zed(self, msg: Odometry):
        # ZED reports a growing pose covariance as visual tracking degrades.
        cov_trace = (msg.pose.covariance[0] + msg.pose.covariance[7]
                     + msg.pose.covariance[35])
        cov_excess = max(0.0, cov_trace - self.zedcov_nominal)
        self.f_zed = clamp(1.0 + self.k_blur * self.turn_rate + self.k_zedcov * cov_excess,
                           1.0, self.max_factor)

        out = self._copy_odom(msg)
        self._set_twist_var(out,
                            vx=self.zed_vx_var * self.f_zed,
                            vy=self.zed_vy_var * self.f_zed,
                            vyaw=self.zed_vyaw_var * self.f_zed)
        self.pub_zed.publish(out)

    # ── helpers ───────────────────────────────────────────────────────────
    @staticmethod
    def _copy_odom(src: Odometry) -> Odometry:
        out = Odometry()
        out.header = src.header
        out.child_frame_id = src.child_frame_id
        out.pose = src.pose
        out.twist = src.twist
        return out

    @staticmethod
    def _set_twist_var(odom: Odometry, vx=None, vy=None, vyaw=None):
        cov = list(odom.twist.covariance)
        big = 1e6
        # diag indices: vx=0, vy=7, vz=14, vroll=21, vpitch=28, vyaw=35
        cov[0] = vx if vx is not None else big
        cov[7] = vy if vy is not None else big
        cov[14] = big
        cov[21] = big
        cov[28] = big
        cov[35] = vyaw if vyaw is not None else big
        odom.twist.covariance = cov

    def _publish_diag(self):
        m = Float32MultiArray()
        m.data = [float(self.slip), float(self.tilt), float(self.turn_rate),
                  float(self.f_wheel), float(self.f_zed)]
        self.pub_diag.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveOdomCovariance()
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
