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
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("slope_filter_alpha", 0.2)
        self.declare_parameter("pitch_threshold", 0.05)
        self.declare_parameter("max_pitch", 0.35)
        self.declare_parameter("max_uphill_factor", 4.5)
        self.declare_parameter("max_downhill_factor", 0.8)
        self.declare_parameter("gps_fix_topic", "/fix")
        self.declare_parameter("gps_vel_topic", "/ublox_gps/fix_velocity")
        self.declare_parameter("imu_topic", "/handsfree/imu")
        self.declare_parameter("gps_heading_speed_threshold_mps", 0.7)
        self.declare_parameter("gps_heading_speed_hysteresis_mps", 0.2)
        self.declare_parameter("gps_heading_alpha", 0.35)
        self.declare_parameter("imu_bias_alpha", 0.1)
        self.declare_parameter("yaw_offset_deg", 0.0)

        self.utm_zone = int(self.get_parameter("utm_zone").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.slope_filter_alpha = float(self.get_parameter("slope_filter_alpha").value)
        self.pitch_threshold = float(self.get_parameter("pitch_threshold").value)
        self.max_pitch = float(self.get_parameter("max_pitch").value)
        self.max_uphill_factor = float(self.get_parameter("max_uphill_factor").value)
        self.max_downhill_factor = float(self.get_parameter("max_downhill_factor").value)
        gps_fix_topic = str(self.get_parameter("gps_fix_topic").value)
        gps_vel_topic = str(self.get_parameter("gps_vel_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        self.gps_heading_speed_threshold = float(self.get_parameter("gps_heading_speed_threshold_mps").value)
        self.gps_heading_speed_hysteresis = float(self.get_parameter("gps_heading_speed_hysteresis_mps").value)
        self.gps_heading_alpha = float(self.get_parameter("gps_heading_alpha").value)
        self.imu_bias_alpha = float(self.get_parameter("imu_bias_alpha").value)
        self.yaw_offset_rad = math.radians(float(self.get_parameter("yaw_offset_deg").value))

        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.gps_speed_mps = 0.0
        self.heading_rad = 0.0
        self.heading_deg = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.imu_yaw = 0.0
        self.imu_yaw_rate = 0.0
        self.imu_heading_bias_rad = 0.0
        self.slope_factor = 1.0
        self.prev_slope_factor = 1.0
        self.has_gps = False
        self.has_imu = False
        self.has_heading = False
        self.has_gps_course_heading = False
        self.has_imu_heading_bias = False
        self.heading_source = "uninitialized"
        self.last_gps_time = self.get_clock().now()
        self.last_gps_vel_time = self.get_clock().now()
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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(NavSatFix, gps_fix_topic, self.gps_cb, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, gps_vel_topic, self.gps_vel_cb, sensor_qos)
        self.create_subscription(Imu, imu_topic, self.imu_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.create_timer(0.02, self.publish_state)

        self._proj = None
        if Proj is not None:
            self._proj = Proj(proj="utm", zone=self.utm_zone, ellps="WGS84", preserve_units=False)
            self.get_logger().info(f"UTM projection enabled (zone={self.utm_zone})")
        else:
            self.get_logger().warn("pyproj not found. Fallback LL->XY conversion will be used.")

        self.get_logger().info(
            f"Status info node started (ROS 2 Humble) | fix={gps_fix_topic} vel={gps_vel_topic} imu={imu_topic}"
        )

    def cmd_cb(self, _: Twist) -> None:
        # Reserved for future compatibility with legacy logic.
        return

    def gps_vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        self.last_gps_vel_time = self.get_clock().now()
        self.vel_x = float(msg.twist.twist.linear.x)
        self.vel_y = float(msg.twist.twist.linear.y)
        self.gps_speed_mps = math.hypot(self.vel_x, self.vel_y)

        if not self.gps_course_is_reliable():
            self.has_gps_course_heading = False
            return

        gps_heading = math.atan2(self.vel_y, self.vel_x)
        self.has_gps_course_heading = True

        if self.has_imu:
            target_bias = self.wrap_angle(gps_heading - self.imu_yaw)
            if not self.has_imu_heading_bias:
                self.imu_heading_bias_rad = target_bias
                self.has_imu_heading_bias = True
            else:
                self.imu_heading_bias_rad = self.interpolate_angle(
                    self.imu_heading_bias_rad, target_bias, self.imu_bias_alpha
                )

        self.update_heading(gps_heading, alpha=self.gps_heading_alpha, source="gps_velocity")

    def imu_cb(self, msg: Imu) -> None:
        self.last_imu_time = self.get_clock().now()
        self.roll, self.pitch, raw_yaw = self.quaternion_to_euler(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        self.imu_yaw = self.wrap_angle(raw_yaw + self.yaw_offset_rad)
        self.imu_yaw_rate = float(msg.angular_velocity.z)
        self.has_imu = True

        if not self.gps_course_is_reliable() and self.has_imu_heading_bias:
            self.update_heading(self.get_imu_aligned_heading(), alpha=1.0, source="imu_fallback")

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
        if not self.has_heading:
            return

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.local_x
        odom.pose.pose.position.y = self.local_y
        odom.pose.pose.position.z = self.local_z
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.heading_rad)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vel_x
        odom.twist.twist.linear.y = self.vel_y
        odom.twist.twist.angular.z = self.imu_yaw_rate
        self.odom_pub.publish(odom)

        heading = Float64()
        heading.data = self.heading_deg
        self.heading_pub.publish(heading)

        slope = Float64()
        slope.data = self.slope_factor
        self.slope_pub.publish(slope)

        if (self.get_clock().now() - self.last_gps_time) > Duration(seconds=1.5):
            self.get_logger().warn("GPS timeout > 1.5s")
        if (self.get_clock().now() - self.last_gps_vel_time) > Duration(seconds=1.5):
            self.get_logger().warn("GPS velocity timeout > 1.5s")
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

    def gps_course_is_reliable(self) -> bool:
        if self.has_gps_course_heading:
            min_speed = max(0.0, self.gps_heading_speed_threshold - self.gps_heading_speed_hysteresis)
        else:
            min_speed = self.gps_heading_speed_threshold
        return self.gps_speed_mps >= min_speed

    def get_imu_aligned_heading(self) -> float:
        if self.has_imu_heading_bias:
            return self.wrap_angle(self.imu_yaw + self.imu_heading_bias_rad)
        return self.imu_yaw

    def update_heading(self, target_heading: float, alpha: float, source: str) -> None:
        target_heading = self.wrap_angle(target_heading)
        if not self.has_heading:
            self.heading_rad = target_heading
            self.has_heading = True
        else:
            self.heading_rad = self.interpolate_angle(self.heading_rad, target_heading, alpha)
        self.heading_deg = math.degrees(self.heading_rad)
        if source != self.heading_source:
            self.heading_source = source
            self.get_logger().info(
                f"Heading source -> {source} (speed={self.gps_speed_mps:.2f} m/s, heading={self.heading_deg:.1f} deg)"
            )

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def interpolate_angle(current: float, target: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        delta = StatusInfoNode.wrap_angle(target - current)
        return StatusInfoNode.wrap_angle(current + alpha * delta)

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
