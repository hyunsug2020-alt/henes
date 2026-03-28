#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class PathFileLoaderNode(Node):
    """Load a saved path text file and publish /global_path."""

    def __init__(self):
        super().__init__('path_file_loader_node')

        self.declare_parameter('path_dir', str(Path.home() / 'henes_ws_ros2' / 'paths'))
        try:
            self.declare_parameter('path_id', '')
        except Exception:
            pass  # already set as integer from CLI/launch
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 0.2)

        path_dir = Path(str(self.get_parameter('path_dir').value)).expanduser()
        path_id = str(self.get_parameter('path_id').value).strip()
        self._path_topic = str(self.get_parameter('path_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if not path_id:
            raise RuntimeError('path_id parameter is empty')

        file_path = self._resolve_path_file(path_dir, path_id)
        if file_path is None:
            raise RuntimeError(f'path file not found for id={path_id} in {path_dir}')

        # Raw rows: list of [x_abs, y_abs, z, yaw, kappa_r]
        self._raw_rows = self._load_raw(file_path)
        if not self._raw_rows:
            raise RuntimeError(f'path file has no valid points: {file_path}')

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_ready = False
        self.path_msg: NavPath = NavPath()

        # Transient local keeps the last /global_path for late subscribers.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(NavPath, self._path_topic, qos)

        # Subscribe to /utm_origin with transient_local to get the UTM offset
        origin_qos = QoSProfile(depth=1)
        origin_qos.reliability = ReliabilityPolicy.RELIABLE
        origin_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Point, '/utm_origin', self._origin_cb, origin_qos)

        self.create_timer(1.0 / max(0.2, publish_rate_hz), self._publish_path)
        self.get_logger().info(
            f'Loaded path id={path_id} points={len(self._raw_rows)} file={file_path}'
        )
        self.get_logger().info('Waiting for /utm_origin...')

    def _origin_cb(self, msg: Point):
        if self._origin_ready:
            return
        self._origin_x = msg.x
        self._origin_y = msg.y
        self._origin_ready = True
        self.path_msg = self._build_path_msg()
        self.get_logger().info(
            f'/utm_origin received ({msg.x:.2f}, {msg.y:.2f}). '
            f'Path ready: {len(self.path_msg.poses)} points.'
        )

    def _resolve_path_file(self, path_dir: Path, path_id: str):
        candidates = [
            path_dir / f'{path_id}_filtered.txt',
            path_dir / f'{path_id}.txt',
            path_dir / f'{path_id}_original.txt',
        ]
        for c in candidates:
            if c.exists():
                return c
        return None

    def _load_raw(self, file_path: Path) -> List[List[float]]:
        """Return list of [x_abs, y_abs, z, yaw, kappa_r] from file."""
        rows = []
        with file_path.open('r', encoding='utf-8') as fp:
            for raw in fp:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) < 2:
                    continue
                try:
                    x     = float(parts[0])
                    y     = float(parts[1])
                    z     = float(parts[2]) if len(parts) >= 3 else 0.0
                    yaw   = float(parts[3]) if len(parts) >= 4 else 0.0
                    kappa = float(parts[4]) if len(parts) >= 5 else 0.0
                except ValueError:
                    continue
                rows.append([x, y, z, yaw, kappa])
        return rows

    def _build_path_msg(self) -> NavPath:
        """Build NavPath from raw rows, subtracting UTM origin from x/y."""
        msg = NavPath()
        msg.header.frame_id = self._frame_id

        for x_abs, y_abs, z, yaw, kappa in self._raw_rows:
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.pose.position.x = x_abs - self._origin_x
            pose.pose.position.y = y_abs - self._origin_y
            pose.pose.position.z = z
            # Encode yaw + kappa_r the same way path_maker does:
            #   orientation.x = kappa_r
            #   orientation.y = 0
            #   orientation.z = sin(yaw/2)
            #   orientation.w = cos(yaw/2)
            pose.pose.orientation.x = kappa
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.poses.append(pose)

        return msg

    def _publish_path(self):
        if not self._origin_ready:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFileLoaderNode()
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
