#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(Node):
    def __init__(self):
        super().__init__('scan_frame_republisher')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_flat')
        self.declare_parameter('output_frame', 'base_laser')
        self.declare_parameter('filtered_odom_topic', '/filtered_odom')
        self.declare_parameter('require_odom_before_publish', True)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.filtered_odom_topic = self.get_parameter('filtered_odom_topic').value
        self.require_odom_before_publish = bool(
            self.get_parameter('require_odom_before_publish').value
        )

        self.first_odom_stamp = None
        self.dropped_before_odom = 0
        self.dropped_before_first_odom_stamp = 0

        self.publisher = self.create_publisher(LaserScan, self.output_scan_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            'Republishing %s as %s in frame %s after %s is available'
            % (
                self.input_scan_topic,
                self.output_scan_topic,
                self.output_frame,
                self.filtered_odom_topic,
            )
        )

    def odom_callback(self, msg):
        if self.first_odom_stamp is None:
            self.first_odom_stamp = msg.header.stamp
            self.get_logger().info(
                'First filtered odom stamp: %d.%09d'
                % (self.first_odom_stamp.sec, self.first_odom_stamp.nanosec)
            )

    def scan_callback(self, msg):
        if self.require_odom_before_publish and self.first_odom_stamp is None:
            self.dropped_before_odom += 1
            if self.dropped_before_odom == 1:
                self.get_logger().info('Dropping scans until filtered odom is available')
            return

        if self.first_odom_stamp is not None and self.stamp_less_equal(
            msg.header.stamp,
            self.first_odom_stamp,
        ):
            self.dropped_before_first_odom_stamp += 1
            if self.dropped_before_first_odom_stamp == 1:
                self.get_logger().info('Dropping startup scans older than first filtered odom')
            return

        republished = LaserScan()
        republished.header = msg.header
        republished.header.frame_id = self.output_frame
        republished.angle_min = msg.angle_min
        republished.angle_max = msg.angle_max
        republished.angle_increment = msg.angle_increment
        republished.time_increment = msg.time_increment
        republished.scan_time = msg.scan_time
        republished.range_min = msg.range_min
        republished.range_max = msg.range_max
        republished.ranges = msg.ranges
        republished.intensities = msg.intensities
        self.publisher.publish(republished)

    def stamp_less_equal(self, left, right):
        if left.sec != right.sec:
            return left.sec < right.sec
        return left.nanosec <= right.nanosec


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRepublisher()
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
