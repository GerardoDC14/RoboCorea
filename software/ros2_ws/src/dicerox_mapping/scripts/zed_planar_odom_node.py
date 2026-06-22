#!/usr/bin/env python3

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class ZedPlanarOdom(Node):
    def __init__(self):
        super().__init__('zed_planar_odom')

        self.declare_parameter('input_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('output_odom_topic', '/filtered_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('zed_x', 0.0)
        self.declare_parameter('zed_y', 0.0)
        self.declare_parameter('zed_yaw', 0.0)
        self.declare_parameter('pose_alpha', 0.35)
        self.declare_parameter('yaw_alpha', 0.25)
        self.declare_parameter('yaw_deadband', 0.01)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('tf_time_offset', 0.0)
        self.declare_parameter('max_pose_jump', 1.0)
        self.declare_parameter('max_yaw_jump', 0.8)
        self.declare_parameter('publish_tf', True)

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.zed_x = float(self.get_parameter('zed_x').value)
        self.zed_y = float(self.get_parameter('zed_y').value)
        self.zed_yaw = float(self.get_parameter('zed_yaw').value)
        self.pose_alpha = clamp(float(self.get_parameter('pose_alpha').value), 0.0, 1.0)
        self.yaw_alpha = clamp(float(self.get_parameter('yaw_alpha').value), 0.0, 1.0)
        self.yaw_deadband = max(0.0, float(self.get_parameter('yaw_deadband').value))
        self.max_yaw_rate = max(0.0, float(self.get_parameter('max_yaw_rate').value))
        self.tf_time_offset = float(self.get_parameter('tf_time_offset').value)
        self.max_pose_jump = float(self.get_parameter('max_pose_jump').value)
        self.max_yaw_jump = float(self.get_parameter('max_yaw_jump').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.filtered_x = None
        self.filtered_y = None
        self.filtered_yaw = None
        self.last_stamp = None

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            20,
        )

        if self.pose_alpha <= 0.05 or self.pose_alpha >= 0.95:
            self.get_logger().warn(
                'pose_alpha=%.2f is near an extreme (0=frozen, 1=raw). '
                'Typical range is 0.1–0.5.' % self.pose_alpha
            )
        if self.yaw_alpha <= 0.05 or self.yaw_alpha >= 0.95:
            self.get_logger().warn(
                'yaw_alpha=%.2f is near an extreme (0=frozen, 1=raw). '
                'Typical range is 0.1–0.4.' % self.yaw_alpha
            )

        self.get_logger().info(
            'Filtering %s into planar %s -> %s odometry'
            % (self.input_odom_topic, self.odom_frame, self.base_frame)
        )

    def odom_callback(self, msg):
        zed_pose = msg.pose.pose
        zed_x = zed_pose.position.x
        zed_y = zed_pose.position.y
        zed_yaw = yaw_from_quaternion(zed_pose.orientation)

        base_yaw = normalize_angle(zed_yaw - self.zed_yaw)
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        base_x = zed_x - (cos_yaw * self.zed_x - sin_yaw * self.zed_y)
        base_y = zed_y - (sin_yaw * self.zed_x + cos_yaw * self.zed_y)

        if self.filtered_x is None:
            self.reset_filter(base_x, base_y, base_yaw)
        else:
            pose_jump = math.hypot(base_x - self.filtered_x, base_y - self.filtered_y)
            yaw_jump = abs(normalize_angle(base_yaw - self.filtered_yaw))
            if pose_jump > self.max_pose_jump or yaw_jump > self.max_yaw_jump:
                self.get_logger().warn(
                    'Resetting planar odom filter after jump: %.3f m, %.3f rad'
                    % (pose_jump, yaw_jump)
                )
                self.reset_filter(base_x, base_y, base_yaw)
            else:
                self.filtered_x += self.pose_alpha * (base_x - self.filtered_x)
                self.filtered_y += self.pose_alpha * (base_y - self.filtered_y)
                yaw_delta = normalize_angle(base_yaw - self.filtered_yaw)
                if abs(yaw_delta) < self.yaw_deadband:
                    yaw_delta = 0.0
                yaw_delta = self.limit_yaw_delta(yaw_delta, msg.header.stamp)
                self.filtered_yaw = normalize_angle(self.filtered_yaw + self.yaw_alpha * yaw_delta)

        self.publish_odom(msg)
        self.last_stamp = msg.header.stamp

    def reset_filter(self, x, y, yaw):
        self.filtered_x = x
        self.filtered_y = y
        self.filtered_yaw = yaw

    def limit_yaw_delta(self, yaw_delta, stamp):
        if self.max_yaw_rate <= 0.0 or self.last_stamp is None:
            return yaw_delta

        current_time = stamp.sec + stamp.nanosec * 1e-9
        last_time = self.last_stamp.sec + self.last_stamp.nanosec * 1e-9
        dt = current_time - last_time
        if dt <= 0.0:
            return yaw_delta

        max_delta = self.max_yaw_rate * dt
        return clamp(yaw_delta, -max_delta, max_delta)

    def offset_stamp(self, stamp, offset_seconds):
        total_nanoseconds = (
            stamp.sec * 1000000000
            + stamp.nanosec
            + int(offset_seconds * 1000000000)
        )
        output = Time()
        output.sec = total_nanoseconds // 1000000000
        output.nanosec = total_nanoseconds % 1000000000
        return output

    def publish_odom(self, source_msg):
        qx, qy, qz, qw = quaternion_from_yaw(self.filtered_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = source_msg.header.stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.filtered_x
        odom_msg.pose.pose.position.y = self.filtered_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist = source_msg.twist.twist

        odom_msg.pose.covariance = list(source_msg.pose.covariance)
        odom_msg.pose.covariance[14] = 1e6
        odom_msg.pose.covariance[21] = 1e6
        odom_msg.pose.covariance[28] = 1e6
        odom_msg.twist.covariance = list(source_msg.twist.covariance)
        odom_msg.twist.covariance[14] = 1e6
        odom_msg.twist.covariance[21] = 1e6
        odom_msg.twist.covariance[28] = 1e6

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = self.offset_stamp(source_msg.header.stamp, self.tf_time_offset)
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.filtered_x
            transform.transform.translation.y = self.filtered_y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ZedPlanarOdom()
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
