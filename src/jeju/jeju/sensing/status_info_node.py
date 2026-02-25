#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Point, Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64

try:
    from pyproj import Proj  # type: ignore
except Exception:
    Proj = None


class StatusInfoNode(Node):
    """ROS 2 Humble status info node: GPS/IMU -> local odometry + heading."""

    def __init__(self):
        super().__init__("status_info_node")

        self.declare_parameter("utm_zone", 52)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("slope_filter_alpha", 0.2)
        self.declare_parameter("pitch_threshold", 0.05)
        self.declare_parameter("max_pitch", 0.35)
        self.declare_parameter("max_uphill_factor", 4.5)
        self.declare_parameter("max_downhill_factor", 0.8)

        self.utm_zone = int(self.get_parameter("utm_zone").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.slope_filter_alpha = float(self.get_parameter("slope_filter_alpha").value)
        self.pitch_threshold = float(self.get_parameter("pitch_threshold").value)
        self.max_pitch = float(self.get_parameter("max_pitch").value)
        self.max_uphill_factor = float(self.get_parameter("max_uphill_factor").value)
        self.max_downhill_factor = float(self.get_parameter("max_downhill_factor").value)

        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.heading_deg = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.slope_factor = 1.0
        self.prev_slope_factor = 1.0
        self.has_gps = False
        self.has_imu = False
        self.last_gps_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.origin_pub = self.create_publisher(Point, "/utm_origin", transient_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.heading_pub = self.create_publisher(Float64, "/ublox_gps/heading", 10)
        self.slope_pub = self.create_publisher(Float64, "/slope_factor", 10)
        self.quality_pub = self.create_publisher(Float64, "/gps/quality", 10)

        self.create_subscription(NavSatFix, "/ublox_gps/fix", self.gps_cb, 20)
        self.create_subscription(TwistWithCovarianceStamped, "/ublox_gps/fix_velocity", self.gps_vel_cb, 20)
        self.create_subscription(Imu, "/ublox_gps/imu", self.imu_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.create_timer(0.02, self.publish_state)

        self._proj = None
        if Proj is not None:
            self._proj = Proj(proj="utm", zone=self.utm_zone, ellps="WGS84", preserve_units=False)
            self.get_logger().info(f"UTM projection enabled (zone={self.utm_zone})")
        else:
            self.get_logger().warn("pyproj not found. Fallback LL->XY conversion will be used.")

        self.get_logger().info("Status info node started (ROS 2 Humble)")

    def cmd_cb(self, _: Twist) -> None:
        # Reserved for future compatibility with legacy logic.
        return

    def gps_vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        self.vel_x = float(msg.twist.twist.linear.x)
        self.vel_y = float(msg.twist.twist.linear.y)

    def imu_cb(self, msg: Imu) -> None:
        self.last_imu_time = self.get_clock().now()
        self.roll, self.pitch, self.yaw = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        self.heading_deg = math.degrees(self.yaw)
        self.has_imu = True

        raw_factor = 1.0
        if abs(self.pitch) > self.pitch_threshold:
            if self.pitch < 0.0:
                normalized_pitch = min(abs(self.pitch) / self.max_pitch, 1.0)
                raw_factor = 1.0 + normalized_pitch * (self.max_uphill_factor - 1.0)
            else:
                normalized_pitch = min(self.pitch / self.max_pitch, 1.0)
                raw_factor = 1.0 - normalized_pitch * (1.0 - self.max_downhill_factor)
        self.slope_factor = self.prev_slope_factor * (1.0 - self.slope_filter_alpha) + raw_factor * self.slope_filter_alpha
        self.prev_slope_factor = self.slope_factor

    def gps_cb(self, msg: NavSatFix) -> None:
        self.last_gps_time = self.get_clock().now()
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        self.local_z = 0.0

        if not self.origin_set:
            self.origin_x = x
            self.origin_y = y
            self.origin_set = True
            origin = Point()
            origin.x = self.origin_x
            origin.y = self.origin_y
            origin.z = 0.0
            self.origin_pub.publish(origin)
            self.get_logger().info(f"UTM origin set: ({self.origin_x:.3f}, {self.origin_y:.3f})")

        self.local_x = x - self.origin_x
        self.local_y = y - self.origin_y
        self.has_gps = True

        quality_msg = Float64()
        quality_msg.data = self.estimate_gps_quality(msg)
        self.quality_pub.publish(quality_msg)

    def publish_state(self) -> None:
        if not self.has_gps:
            return

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.local_x
        odom.pose.pose.position.y = self.local_y
        odom.pose.pose.position.z = self.local_z
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, math.radians(self.heading_deg))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vel_x
        odom.twist.twist.linear.y = self.vel_y
        self.odom_pub.publish(odom)

        heading = Float64()
        heading.data = self.heading_deg
        self.heading_pub.publish(heading)

        slope = Float64()
        slope.data = self.slope_factor
        self.slope_pub.publish(slope)

        if (self.get_clock().now() - self.last_gps_time) > Duration(seconds=1.5):
            self.get_logger().warn("GPS timeout > 1.5s")
        if self.has_gps and not self.has_imu and (self.get_clock().now() - self.last_imu_time) > Duration(seconds=1.5):
            self.get_logger().warn("IMU timeout > 1.5s")

    def estimate_gps_quality(self, msg: NavSatFix) -> float:
        if len(msg.position_covariance) < 1:
            return 0.5
        variance = float(msg.position_covariance[0])
        if variance <= 0.0:
            return 0.7
        if variance < 0.05:
            return 0.95
        if variance < 0.2:
            return 0.8
        if variance < 1.0:
            return 0.65
        return 0.4

    def latlon_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if self._proj is not None:
            x, y = self._proj(lon, lat)
            return float(x), float(y)
        return self.fallback_ll_to_xy(lat, lon)

    @staticmethod
    def fallback_ll_to_xy(lat: float, lon: float) -> Tuple[float, float]:
        # Equirectangular approximation for fallback only.
        r = 6378137.0
        x = math.radians(lon) * r * math.cos(math.radians(lat))
        y = math.radians(lat) * r
        return x, y

    @staticmethod
    def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
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
    node = StatusInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
