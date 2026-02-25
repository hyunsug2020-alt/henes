#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Int32


class WheelOdomNode(Node):
    """Compute wheel odometry from encoder counts."""

    def __init__(self):
        super().__init__('wheel_odom_node')

        self.declare_parameter('left_encoder_topic', '/encoder1')
        self.declare_parameter('right_encoder_topic', '/encoder2')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('counts_per_rev', 4096.0)
        self.declare_parameter('wheel_radius_m', 0.13)
        self.declare_parameter('wheel_base_m', 0.70)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('imu_heading_topic', '/ublox_gps/heading')
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        self.left_topic = str(self.get_parameter('left_encoder_topic').value)
        self.right_topic = str(self.get_parameter('right_encoder_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.counts_per_rev = float(self.get_parameter('counts_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)
        self.imu_heading_topic = str(self.get_parameter('imu_heading_topic').value)
        self.left_sign = float(self.get_parameter('left_sign').value)
        self.right_sign = float(self.get_parameter('right_sign').value)

        self.left_count = 0
        self.right_count = 0
        self.prev_left_count = None
        self.prev_right_count = None
        self.has_left = False
        self.has_right = False
        self.has_imu_heading = False
        self.imu_heading_rad = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.create_subscription(Int32, self.left_topic, self.left_cb, 20)
        self.create_subscription(Int32, self.right_topic, self.right_cb, 20)
        self.create_subscription(Float64, self.imu_heading_topic, self.imu_heading_cb, 20)

        hz = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self.step)

        self.get_logger().info(
            f'Wheel odom started: {self.left_topic}, {self.right_topic} -> {self.odom_topic} '
            f'(cpr={self.counts_per_rev}, r={self.wheel_radius}, base={self.wheel_base})'
        )

    def left_cb(self, msg: Int32) -> None:
        self.left_count = int(msg.data)
        self.has_left = True

    def right_cb(self, msg: Int32) -> None:
        self.right_count = int(msg.data)
        self.has_right = True

    def imu_heading_cb(self, msg: Float64) -> None:
        self.imu_heading_rad = math.radians(float(msg.data))
        self.has_imu_heading = True

    def step(self) -> None:
        if not (self.has_left and self.has_right):
            return
        if self.prev_left_count is None or self.prev_right_count is None:
            self.prev_left_count = self.left_count
            self.prev_right_count = self.right_count
            self.last_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 1e-6:
            return

        d_left_cnt = (self.left_count - self.prev_left_count) * self.left_sign
        d_right_cnt = (self.right_count - self.prev_right_count) * self.right_sign
        self.prev_left_count = self.left_count
        self.prev_right_count = self.right_count
        self.last_time = now

        meters_per_count = (2.0 * math.pi * self.wheel_radius) / max(self.counts_per_rev, 1.0)
        d_left = d_left_cnt * meters_per_count
        d_right = d_right_cnt * meters_per_count

        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / max(self.wheel_base, 1e-6)

        if self.use_imu_heading and self.has_imu_heading:
            self.yaw = self.imu_heading_rad
        else:
            self.yaw += dtheta

        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dtheta / dt
        self.odom_pub.publish(odom)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
