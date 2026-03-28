#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class TraveledPathNode(Node):
    def __init__(self):
        super().__init__('traveled_path_node')
        self.declare_parameter('min_dist_m', 0.05)

        self.min_dist = self.get_parameter('min_dist_m').value

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self._last_x = None
        self._last_y = None

        self.sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.pub = self.create_publisher(Path, '/traveled_path', 10)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_x is not None:
            if math.hypot(x - self._last_x, y - self._last_y) < self.min_dist:
                return

        self._last_x = x
        self._last_y = y

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = msg.header.stamp
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TraveledPathNode()
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
