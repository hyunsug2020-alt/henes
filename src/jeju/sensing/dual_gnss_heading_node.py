#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64
from ublox_ubx_msgs.msg import UBXNavRelPosNED


class DualGnssHeadingNode(Node):
    """Publish vehicle heading from a dual-F9P RELPOSNED stream."""

    def __init__(self):
        super().__init__("dual_gnss_heading_node")

        self.declare_parameter("relpos_topic", "/gnss_rover/ubx_nav_rel_pos_ned")
        self.declare_parameter("heading_topic", "/dual_f9p/heading")
        self.declare_parameter("heading_accuracy_topic", "/dual_f9p/heading_accuracy_deg")
        self.declare_parameter("baseline_length_topic", "/dual_f9p/baseline_length_m")
        self.declare_parameter("heading_valid_topic", "/dual_f9p/heading_valid")
        self.declare_parameter("min_carr_soln_status", 2)
        self.declare_parameter("require_relpos_valid", True)
        self.declare_parameter("require_heading_valid", True)
        self.declare_parameter("heading_offset_deg", 0.0)
        self.declare_parameter("heading_alpha", 0.35)

        relpos_topic = str(self.get_parameter("relpos_topic").value)
        heading_topic = str(self.get_parameter("heading_topic").value)
        heading_accuracy_topic = str(self.get_parameter("heading_accuracy_topic").value)
        baseline_length_topic = str(self.get_parameter("baseline_length_topic").value)
        heading_valid_topic = str(self.get_parameter("heading_valid_topic").value)
        self.min_carr_soln_status = int(self.get_parameter("min_carr_soln_status").value)
        self.require_relpos_valid = bool(self.get_parameter("require_relpos_valid").value)
        self.require_heading_valid = bool(self.get_parameter("require_heading_valid").value)
        self.heading_offset_deg = float(self.get_parameter("heading_offset_deg").value)
        self.heading_alpha = float(self.get_parameter("heading_alpha").value)

        pub_qos = QoSProfile(depth=10)
        self.heading_pub = self.create_publisher(Float64, heading_topic, pub_qos)
        self.heading_acc_pub = self.create_publisher(Float64, heading_accuracy_topic, pub_qos)
        self.baseline_pub = self.create_publisher(Float64, baseline_length_topic, pub_qos)
        self.valid_pub = self.create_publisher(Bool, heading_valid_topic, pub_qos)

        ubx_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(UBXNavRelPosNED, relpos_topic, self.relpos_cb, ubx_qos)

        self.has_heading = False
        self.heading_deg = 0.0
        self.last_valid = None

        self.get_logger().info(
            f"Dual GNSS heading node started | relpos={relpos_topic} -> heading={heading_topic}"
        )

    def relpos_cb(self, msg: UBXNavRelPosNED) -> None:
        valid = True
        if self.require_relpos_valid and not msg.rel_pos_valid:
            valid = False
        if self.require_heading_valid and not msg.rel_pos_heading_valid:
            valid = False
        if int(msg.carr_soln.status) < self.min_carr_soln_status:
            valid = False

        baseline_cm = float(msg.rel_pos_length) + float(msg.rel_pos_hp_length) * 1e-2
        baseline_m = baseline_cm * 1e-2
        heading_accuracy_deg = float(msg.acc_heading) * 1e-5

        baseline_msg = Float64()
        baseline_msg.data = baseline_m
        self.baseline_pub.publish(baseline_msg)

        valid_msg = Bool()
        valid_msg.data = valid
        self.valid_pub.publish(valid_msg)

        if self.last_valid is None or self.last_valid != valid:
            self.last_valid = valid
            state = "valid" if valid else "invalid"
            self.get_logger().info(
                f"RELPOS heading became {state} "
                f"(carr={int(msg.carr_soln.status)}, rel_valid={msg.rel_pos_valid}, "
                f"heading_valid={msg.rel_pos_heading_valid}, baseline={baseline_m:.3f} m)"
            )

        if not valid:
            return

        target_heading_deg = self.wrap_deg(float(msg.rel_pos_heading) * 1e-5 + self.heading_offset_deg)
        if self.has_heading:
            self.heading_deg = self.interpolate_deg(self.heading_deg, target_heading_deg, self.heading_alpha)
        else:
            self.heading_deg = target_heading_deg
            self.has_heading = True

        heading_msg = Float64()
        heading_msg.data = self.heading_deg
        self.heading_pub.publish(heading_msg)

        heading_acc_msg = Float64()
        heading_acc_msg.data = heading_accuracy_deg
        self.heading_acc_pub.publish(heading_acc_msg)

    @staticmethod
    def wrap_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def interpolate_deg(current_deg: float, target_deg: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        delta = DualGnssHeadingNode.wrap_deg(target_deg - current_deg)
        return DualGnssHeadingNode.wrap_deg(current_deg + alpha * delta)


def main(args=None):
    rclpy.init(args=args)
    node = DualGnssHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
