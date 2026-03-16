#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from mgeo_class_defs import MGeoPlannerMap


class HDMapVisualizer(Node):
    def __init__(self):
        super().__init__("hdmap_visualizer")

        # self.get_logger().info("="*60)
        # self.get_logger().info("HD Map Visualizer (Frame: world)")
        # self.get_logger().info("="*60)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.link_marker_pub = self.create_publisher(
            MarkerArray, "/hdmap/link_markers", qos
        )
        self.node_marker_pub = self.create_publisher(
            MarkerArray, "/hdmap/node_markers", qos
        )

        # HDMap 경로 (install 우선)
        try:
            package_share = get_package_share_directory("bisa")
            load_path = os.path.join(package_share, "hdmap_data")
            # self.get_logger().info(f"Loading HD Map from: {load_path}")
        except:
            # Fallback: 소스 디렉토리
            workspace_src = os.path.join(
                os.path.expanduser("~"), "Mobility_Challenge_Simulator", "src", "bisa"
            )
            load_path = os.path.join(workspace_src, "hdmap_data")
            # self.get_logger().warn(f"Using source directory: {load_path}")

        if not os.path.exists(load_path):
            # self.get_logger().error("\nHD map not found!")
            # self.get_logger().error("Please run: ros2 run bisa hdmap_generator.py\n")
            sys.exit(1)

        mgeo = MGeoPlannerMap.create_instance_from_json(load_path)
        self.nodes = mgeo.node_set.nodes
        self.links = mgeo.link_set.lines

        self.link_markers = self.create_link_markers()
        self.node_markers = self.create_node_markers()

        # self.get_logger().info(f"Total Nodes: {len(self.nodes)}")
        # self.get_logger().info(f"Total Links: {len(self.links)}")
        # self.get_logger().info("Publishing HD Map (Frame: world)\n")

        self.timer = self.create_timer(10.0, self.publish_hdmap)
        self.publish_hdmap()

    def create_link_markers(self):
        """링크를 작은 점들로 표시"""
        markers = MarkerArray()

        for i, link in enumerate(self.links.values()):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "links"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD

            for point in link.points:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2])
                marker.points.append(p)

            marker.scale.x = 0.008
            marker.scale.y = 0.008

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8

            markers.markers.append(marker)

        return markers

    def create_node_markers(self):
        """노드를 작은 구체로 표시"""
        markers = MarkerArray()

        for i, node in enumerate(self.nodes.values()):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(node.point[0])
            marker.pose.position.y = float(node.point[1])
            marker.pose.position.z = float(node.point[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            markers.markers.append(marker)

        return markers

    def publish_hdmap(self):
        self.link_marker_pub.publish(self.link_markers)
        self.node_marker_pub.publish(self.node_markers)


def main(args=None):
    rclpy.init(args=args)
    node = HDMapVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
