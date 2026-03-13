#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from ublox_ubx_msgs.msg import UBXNavVelNED


class GnssVelocityBridgeNode(Node):
    """Bridge UBX NAV-VELNED into the legacy /ublox_gps/fix_velocity topic."""

    def __init__(self) -> None:
        super().__init__("gnss_velocity_bridge_node")

        self.declare_parameter("input_topic", "/gnss_rover/ubx_nav_vel_ned")
        self.declare_parameter("output_topic", "/ublox_gps/fix_velocity")
        self.declare_parameter("frame_id", "odom")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pub = self.create_publisher(TwistWithCovarianceStamped, output_topic, 20)
        self.create_subscription(UBXNavVelNED, input_topic, self.vel_cb, 20)

        self.get_logger().info(
            f"GNSS velocity bridge started | {input_topic} -> {output_topic}"
        )

    def vel_cb(self, msg: UBXNavVelNED) -> None:
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        if self.frame_id:
            out.header.frame_id = self.frame_id

        # UBX NAV-VELNED is North/East/Down in cm/s. Local odom is East/North,
        # so map x=east, y=north to keep atan2(vy, vx) consistent with /odometry/filtered.
        out.twist.twist.linear.x = float(msg.vel_e) * 0.01
        out.twist.twist.linear.y = float(msg.vel_n) * 0.01
        out.twist.twist.linear.z = -float(msg.vel_d) * 0.01

        speed_sigma = float(msg.s_acc) * 0.01
        variance = speed_sigma * speed_sigma
        out.twist.covariance[0] = variance
        out.twist.covariance[7] = variance
        out.twist.covariance[14] = variance

        heading_sigma = math.radians(float(msg.c_acc) * 1e-5)
        out.twist.covariance[35] = heading_sigma * heading_sigma

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssVelocityBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
