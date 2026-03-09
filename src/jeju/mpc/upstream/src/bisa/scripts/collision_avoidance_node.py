#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
충돌 방지 노드 v15.0

수정:
1. HV 판단 - 단순화 + 디버그 로그
2. 회전교차로 진입 전 충분히 일찍 판단
3. HV가 회전교차로에 있고 + 내 경로와 가까우면 → STOP
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Accel
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from bisa.msg import LapInfo
import math
from collections import deque
import time


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")

        # self.get_logger().info("="*60)
        # self.get_logger().info("Collision Avoidance Node v15.0")
        # self.get_logger().info("Simplified HV check with debug logs")
        # self.get_logger().info("="*60)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # CAV 색상
        self.cav_colors = {
            1: {"r": 1.0, "g": 0.0, "b": 0.0},
            2: {"r": 0.0, "g": 0.0, "b": 1.0},
            3: {"r": 0.0, "g": 1.0, "b": 0.0},
            4: {"r": 1.0, "g": 1.0, "b": 0.0},
        }

        # 우선순위
        self.high_priority_cavs = [1, 2]
        self.low_priority_cavs = [3, 4]

        # ===== 회전교차로 =====
        self.roundabout_center = (0.5741, -0.2909)
        self.roundabout_radius = 1.9

        # 진입 판단 거리 (회전교차로 중심에서)
        self.entry_check_radius = 2.5  # 이 반경 안에 들어오면 HV 체크 시작

        # ===== HV 파라미터 (단순화) =====
        self.hv_danger_dist = 1.2  # HV가 내 경로에서 이 거리 이내면 위험
        self.hv_check_ahead = 100  # 내 경로에서 확인할 포인트 수

        # ===== CAV 충돌 파라미터 =====
        self.path_cross_dist = 0.20
        self.collision_radius = 0.25
        self.time_window = 20
        self.safe_stop_dist = 0.45
        self.early_slow_dist = 1.0

        self.lookahead_points = 100
        self.max_velocity = 1.2
        self.min_velocity = 0.4
        self.accel_rate = 0.08
        self.decel_rate = 0.12

        # HV 상태
        self.hv_ids = [19, 20]
        self.hv_poses = {}
        self.hv_velocities = {}
        self.hv_history = {}

        for hv_id in self.hv_ids:
            self.hv_poses[hv_id] = None
            self.hv_velocities[hv_id] = (0.0, 0.0, 0.0)
            self.hv_history[hv_id] = deque(maxlen=15)

            self.create_subscription(
                PoseStamped,
                f"/HV_{hv_id}",
                lambda msg, hid=hv_id: self.hv_pose_callback(msg, hid),
                sensor_qos,
            )

        # CAV 상태
        self.cav_ids = [1, 2, 3, 4]
        self.cav_poses = {}
        self.cav_yaws = {}
        self.cav_stop_flags = {}
        self.cav_slow_flags = {}
        self.cav_local_paths = {}
        self.cav_remaining = {}
        self.cav_current_velocity = {}
        self.cav_stop_reasons = {}

        for cav_id in self.cav_ids:
            self.cav_poses[cav_id] = None
            self.cav_yaws[cav_id] = 0.0
            self.cav_stop_flags[cav_id] = False
            self.cav_slow_flags[cav_id] = False
            self.cav_local_paths[cav_id] = []
            self.cav_remaining[cav_id] = 9999
            self.cav_current_velocity[cav_id] = 0.0
            self.cav_stop_reasons[cav_id] = ""

            self.create_subscription(
                PoseStamped,
                f"/CAV_{cav_id:02d}",
                lambda msg, cid=cav_id: self.cav_pose_callback(msg, cid),
                sensor_qos,
            )

            self.create_subscription(
                Path,
                f"/local_path_cav{cav_id:02d}",
                lambda msg, cid=cav_id: self.local_path_callback(msg, cid),
                10,
            )

            self.create_subscription(
                LapInfo,
                f"/lap_info_cav{cav_id:02d}",
                lambda msg, cid=cav_id: self.lap_info_callback(msg, cid),
                10,
            )

        # Accel 중계
        self.accel_pubs = {}
        self.last_accel = {}

        for cav_id in self.cav_ids:
            self.create_subscription(
                Accel,
                f"/CAV_{cav_id:02d}_accel_raw",
                lambda msg, cid=cav_id: self.accel_callback(msg, cid),
                10,
            )
            self.accel_pubs[cav_id] = self.create_publisher(
                Accel, f"/CAV_{cav_id:02d}_accel", 10
            )
            self.last_accel[cav_id] = Accel()

        # 시각화
        vis_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hv_marker_pub = self.create_publisher(MarkerArray, "/hv_markers", vis_qos)
        self.cav_marker_pub = self.create_publisher(
            MarkerArray, "/cav_markers", vis_qos
        )
        self.zone_marker_pub = self.create_publisher(
            MarkerArray, "/collision_zones", vis_qos
        )

        self.active_collision_zones = []
        self.hv_debug_info = {}  # 디버그용

        # 타이머
        self.timer = self.create_timer(0.001, self.control_loop)
        self.vis_timer = self.create_timer(0.1, self.publish_visualizations)
        self.log_timer = self.create_timer(2.0, self.log_status)  # 2초마다
        self.debug_timer = self.create_timer(1.0, self.log_hv_debug)  # 1초마다 HV 디버그

        # self.get_logger().info("✓ Ready!")

    # ==================== 콜백 ====================

    def hv_pose_callback(self, msg, hv_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = msg.pose.orientation.z

        current_time = time.time()

        if len(self.hv_history[hv_id]) >= 2:
            prev_time, prev_x, prev_y = self.hv_history[hv_id][-1]
            dt = current_time - prev_time

            if dt > 0.01:
                vx = (x - prev_x) / dt
                vy = (y - prev_y) / dt
                speed = math.sqrt(vx**2 + vy**2)

                old_vx, old_vy, _ = self.hv_velocities[hv_id]
                alpha = 0.3
                vx = alpha * vx + (1 - alpha) * old_vx
                vy = alpha * vy + (1 - alpha) * old_vy
                speed = math.sqrt(vx**2 + vy**2)

                self.hv_velocities[hv_id] = (vx, vy, speed)

        self.hv_history[hv_id].append((current_time, x, y))
        self.hv_poses[hv_id] = (x, y, yaw)

    def cav_pose_callback(self, msg, cav_id):
        self.cav_poses[cav_id] = (msg.pose.position.x, msg.pose.position.y)
        self.cav_yaws[cav_id] = msg.pose.orientation.z

    def local_path_callback(self, msg, cav_id):
        self.cav_local_paths[cav_id] = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses[: self.lookahead_points]
        ]

    def lap_info_callback(self, msg, cav_id):
        self.cav_remaining[cav_id] = msg.total_waypoints - msg.current_waypoint

    def accel_callback(self, msg, cav_id):
        self.last_accel[cav_id] = msg

    # ==================== 유틸리티 ====================

    def dist(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def is_in_roundabout(self, x, y):
        return self.dist((x, y), self.roundabout_center) < self.roundabout_radius

    def is_near_roundabout(self, x, y):
        """회전교차로 근처인지 (진입 체크 영역)"""
        return self.dist((x, y), self.roundabout_center) < self.entry_check_radius

    # ==================== HV 충돌 검사 (단순화 + 디버그) ====================

    def check_hv_collision(self, cav_id):
        """
        HV 충돌 검사 - 단순화 버전

        로직:
        1. 내가 회전교차로 근처에 있는지 확인
        2. 내 경로가 회전교차로를 통과하는지 확인
        3. HV가 회전교차로에 있는지 확인
        4. HV가 내 (미래) 경로와 가까운지 확인
        """
        my_path = self.cav_local_paths.get(cav_id, [])
        my_pos = self.cav_poses.get(cav_id)

        if not my_path or my_pos is None:
            return False, None

        my_x, my_y = my_pos

        # 1. 이미 회전교차로 안이면 검사 안함 (나가야 함)
        if self.is_in_roundabout(my_x, my_y):
            self.hv_debug_info[cav_id] = "Already in RB - skip"
            return False, None

        # 2. 내 경로가 회전교차로를 통과하는지
        path_enters_rb = False
        entry_idx = -1
        for i, (px, py) in enumerate(my_path[: self.hv_check_ahead]):
            if self.is_in_roundabout(px, py):
                path_enters_rb = True
                entry_idx = i
                break

        if not path_enters_rb:
            self.hv_debug_info[cav_id] = "Path doesn't enter RB"
            return False, None

        # 3. 내가 회전교차로 근처(진입 체크 영역)에 있는지
        if not self.is_near_roundabout(my_x, my_y):
            self.hv_debug_info[cav_id] = f"Too far from RB (entry_idx={entry_idx})"
            return False, None

        # 4. HV 검사
        for hv_id in self.hv_ids:
            if self.hv_poses[hv_id] is None:
                continue

            hv_x, hv_y, _ = self.hv_poses[hv_id]
            hv_vx, hv_vy, hv_speed = self.hv_velocities[hv_id]

            # HV가 회전교차로 안에 있는지
            if not self.is_in_roundabout(hv_x, hv_y):
                continue

            # 5. 내 경로(회전교차로 구간)와 HV 현재 위치/미래 위치 비교
            # 간단하게: HV 현재 위치 + 1~3초 후 예측 위치와 내 경로 비교

            for t in [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
                # HV 예측 위치 (선형 + 회전 보정)
                # 회전교차로에서는 원형으로 돌기 때문에 단순 선형은 부정확
                # 대신 현재 속도 방향으로 예측
                hv_future_x = hv_x + hv_vx * t
                hv_future_y = hv_y + hv_vy * t

                # 내 경로(회전교차로 구간)와 비교
                for i in range(entry_idx, min(entry_idx + 60, len(my_path))):
                    px, py = my_path[i]

                    d = self.dist((px, py), (hv_future_x, hv_future_y))

                    if d < self.hv_danger_dist:
                        self.hv_debug_info[
                            cav_id
                        ] = f"DANGER! HV{hv_id} t={t:.1f}s d={d:.2f}m path_idx={i}"
                        return True, hv_id

            # HV가 회전교차로에 있지만 내 경로와 안 겹침
            self.hv_debug_info[cav_id] = f"HV{hv_id} in RB but path clear"

        self.hv_debug_info[cav_id] = "No HV danger"
        return False, None

    # ==================== CAV 충돌 검사 ====================

    def find_real_collision(self, path1, path2):
        """실제 충돌 지점 찾기"""
        if len(path1) < 20 or len(path2) < 20:
            return False, None, -1, -1

        n = min(len(path1), len(path2), 120)

        for i in range(0, n, 4):
            p1 = path1[i]

            j_start = max(0, i - self.time_window)
            j_end = min(len(path2), i + self.time_window + 1)

            for j in range(j_start, j_end, 3):
                p2 = path2[j]
                d = self.dist(p1, p2)

                if d < self.path_cross_dist:
                    # 방향 검사
                    if (
                        i >= 12
                        and j >= 12
                        and i < len(path1) - 3
                        and j < len(path2) - 3
                    ):
                        dx1 = path1[i][0] - path1[i - 12][0]
                        dy1 = path1[i][1] - path1[i - 12][1]
                        dx1b = path1[min(i + 3, len(path1) - 1)][0] - path1[i][0]
                        dy1b = path1[min(i + 3, len(path1) - 1)][1] - path1[i][1]
                        dx1 = dx1 + dx1b
                        dy1 = dy1 + dy1b

                        dx2 = path2[j][0] - path2[j - 12][0]
                        dy2 = path2[j][1] - path2[j - 12][1]
                        dx2b = path2[min(j + 3, len(path2) - 1)][0] - path2[j][0]
                        dy2b = path2[min(j + 3, len(path2) - 1)][1] - path2[j][1]
                        dx2 = dx2 + dx2b
                        dy2 = dy2 + dy2b

                        len1 = math.sqrt(dx1**2 + dy1**2)
                        len2 = math.sqrt(dx2**2 + dy2**2)

                        if len1 > 0.02 and len2 > 0.02:
                            dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2)
                            dot = max(-1, min(1, dot))

                            if abs(dot) > 0.5:
                                continue

                    cx = (p1[0] + p2[0]) / 2
                    cy = (p1[1] + p2[1]) / 2
                    return True, (cx, cy), i, j

        return False, None, -1, -1

    def get_path_distance(self, path, start_idx, end_idx):
        if not path or start_idx >= end_idx:
            return 0
        total = 0
        for i in range(start_idx, min(end_idx, len(path) - 1)):
            total += self.dist(path[i], path[i + 1])
        return total

    def find_closest_path_idx(self, pos, path):
        if not path or pos is None:
            return 0
        min_d = float("inf")
        min_idx = 0
        for i, p in enumerate(path[:60]):
            d = self.dist(pos, p)
            if d < min_d:
                min_d = d
                min_idx = i
        return min_idx

    def check_cav_collision(self, cav_id):
        """CAV 간 충돌 검사"""
        my_path = self.cav_local_paths.get(cav_id, [])
        my_pos = self.cav_poses.get(cav_id)

        if not my_path or my_pos is None:
            return False, None, False

        my_priority = 2 if cav_id in self.high_priority_cavs else 1
        my_remaining = self.cav_remaining.get(cav_id, 0)

        for other_id in self.cav_ids:
            if other_id == cav_id:
                continue

            other_path = self.cav_local_paths.get(other_id, [])
            other_pos = self.cav_poses.get(other_id)

            if not other_path or other_pos is None:
                continue

            has_collision, zone, my_idx, other_idx = self.find_real_collision(
                my_path, other_path
            )

            if not has_collision:
                continue

            if zone:
                self.active_collision_zones = [zone]

            i_am_in = self.dist(my_pos, zone) < self.collision_radius if zone else False
            other_in = (
                self.dist(other_pos, zone) < self.collision_radius if zone else False
            )

            if other_in and not i_am_in:
                return True, other_id, False

            if i_am_in:
                continue

            my_dist = self.get_path_distance(
                my_path, self.find_closest_path_idx(my_pos, my_path), my_idx
            )

            other_priority = 2 if other_id in self.high_priority_cavs else 1
            other_remaining = self.cav_remaining.get(other_id, 0)

            i_should_yield = False
            if other_priority > my_priority:
                i_should_yield = True
            elif other_priority == my_priority:
                if other_remaining > my_remaining:
                    i_should_yield = True
                elif other_remaining == my_remaining and other_id < cav_id:
                    i_should_yield = True

            if i_should_yield:
                if my_dist < self.safe_stop_dist:
                    return True, other_id, False
                elif my_dist < self.early_slow_dist:
                    return False, other_id, True

        return False, None, False

    # ==================== 제어 ====================

    def smooth_velocity(self, cav_id, target):
        current = self.cav_current_velocity[cav_id]
        if target > current:
            new_vel = min(current + self.accel_rate, target)
        else:
            new_vel = max(current - self.decel_rate, target)
        self.cav_current_velocity[cav_id] = new_vel
        return new_vel

    def control_loop(self):
        for cav_id in self.cav_ids:
            should_stop = False
            should_slow = False
            reason = ""

            # 1. HV 검사
            hv_danger, hv_id = self.check_hv_collision(cav_id)
            if hv_danger:
                should_stop = True
                reason = f"HV{hv_id}"

            # 2. CAV 검사
            if not should_stop:
                cav_stop, other_id, cav_slow = self.check_cav_collision(cav_id)
                if cav_stop:
                    should_stop = True
                    reason = f"CAV{other_id:02d}"
                elif cav_slow and not should_slow:
                    should_slow = True
                    reason = f"slow:CAV{other_id:02d}"

            # 로그
            # if should_stop != self.cav_stop_flags[cav_id]:
            #     if should_stop:
            #         self.get_logger().warn(f"⛔ CAV{cav_id:02d} STOP: {reason}")
            #     else:
            #         self.get_logger().info(f"✅ CAV{cav_id:02d} GO")

            self.cav_stop_flags[cav_id] = should_stop
            self.cav_slow_flags[cav_id] = should_slow
            self.cav_stop_reasons[cav_id] = reason

            # 속도 명령
            accel_msg = Accel()
            raw = self.last_accel[cav_id]

            if should_stop:
                target_v = 0.0
            elif should_slow:
                target_v = self.min_velocity
            else:
                target_v = raw.linear.x

            smooth_v = self.smooth_velocity(cav_id, target_v)
            accel_msg.linear.x = smooth_v
            accel_msg.angular.z = raw.angular.z if smooth_v > 0.05 else 0.0

            self.accel_pubs[cav_id].publish(accel_msg)

    # ==================== 디버그 로그 ====================

    def log_hv_debug(self):
        """HV 디버그 정보 출력"""
        # HV 상태
        for hv_id in self.hv_ids:
            if self.hv_poses[hv_id]:
                hv_x, hv_y, _ = self.hv_poses[hv_id]
                _, _, speed = self.hv_velocities[hv_id]
                in_rb = "🔵RB" if self.is_in_roundabout(hv_x, hv_y) else "OUT"
                # self.get_logger().info(f"HV{hv_id}: ({hv_x:.2f},{hv_y:.2f}) {speed:.2f}m/s {in_rb}")

        # CAV별 HV 판단 결과
        for cav_id in self.cav_ids:
            debug = self.hv_debug_info.get(cav_id, "N/A")
            # self.get_logger().info(f"  CAV{cav_id:02d} HV check: {debug}")

    # ==================== 시각화 ====================

    def publish_visualizations(self):
        self.publish_hv_markers()
        self.publish_cav_markers()
        self.publish_zone_markers()

    def publish_hv_markers(self):
        markers = MarkerArray()
        for i, hv_id in enumerate(self.hv_ids):
            if self.hv_poses[hv_id] is None:
                continue
            x, y, yaw = self.hv_poses[hv_id]

            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "hv"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            m.scale.x = 0.33
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 0.6
            m.color.g = 0.6
            m.color.b = 0.6
            m.color.a = 0.95
            markers.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = "hv_label"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.35
            label.scale.z = 0.12
            _, _, speed = self.hv_velocities[hv_id]
            in_rb = "🔵" if self.is_in_roundabout(x, y) else ""
            label.text = f"HV{hv_id}{in_rb}\n{speed:.2f}m/s"
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            markers.markers.append(label)

        self.hv_marker_pub.publish(markers)

    def publish_cav_markers(self):
        markers = MarkerArray()
        for cav_id in self.cav_ids:
            if self.cav_poses[cav_id] is None:
                continue

            x, y = self.cav_poses[cav_id]
            yaw = self.cav_yaws[cav_id]
            color = self.cav_colors[cav_id]
            curr_vel = self.cav_current_velocity.get(cav_id, 0)

            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "cav"
            m.id = cav_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            m.scale.x = 0.33
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = color["r"]
            m.color.g = color["g"]
            m.color.b = color["b"]
            m.color.a = 0.95
            markers.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = "cav_label"
            label.id = cav_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.35
            label.scale.z = 0.08

            if self.cav_stop_flags[cav_id]:
                status = "STOP"
            elif self.cav_slow_flags[cav_id]:
                status = "SLOW"
            else:
                status = "GO"

            pri = "H" if cav_id in self.high_priority_cavs else "L"
            label.text = f"CAV{cav_id:02d}[{pri}]\n{status} v:{curr_vel:.2f}"

            if self.cav_stop_flags[cav_id]:
                label.color.r = 1.0
                label.color.g = 0.3
                label.color.b = 0.3
            elif self.cav_slow_flags[cav_id]:
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 0.3
            else:
                label.color.r = 0.3
                label.color.g = 1.0
                label.color.b = 0.3
            label.color.a = 1.0
            markers.markers.append(label)

        self.cav_marker_pub.publish(markers)

    def publish_zone_markers(self):
        markers = MarkerArray()

        # 충돌 구간
        for i, (cx, cy) in enumerate(self.active_collision_zones):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "collision_zone"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = self.collision_radius * 2
            m.scale.y = self.collision_radius * 2
            m.scale.z = 0.02
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.5
            markers.markers.append(m)

        # 회전교차로 영역 표시
        rb = Marker()
        rb.header.frame_id = "world"
        rb.header.stamp = self.get_clock().now().to_msg()
        rb.ns = "roundabout"
        rb.id = 0
        rb.type = Marker.CYLINDER
        rb.action = Marker.ADD
        rb.pose.position.x = self.roundabout_center[0]
        rb.pose.position.y = self.roundabout_center[1]
        rb.pose.position.z = 0.005
        rb.pose.orientation.w = 1.0
        rb.scale.x = self.roundabout_radius * 2
        rb.scale.y = self.roundabout_radius * 2
        rb.scale.z = 0.01
        rb.color.r = 0.0
        rb.color.g = 0.5
        rb.color.b = 1.0
        rb.color.a = 0.15
        markers.markers.append(rb)

        # 진입 체크 영역
        ec = Marker()
        ec.header.frame_id = "world"
        ec.header.stamp = self.get_clock().now().to_msg()
        ec.ns = "entry_check"
        ec.id = 0
        ec.type = Marker.CYLINDER
        ec.action = Marker.ADD
        ec.pose.position.x = self.roundabout_center[0]
        ec.pose.position.y = self.roundabout_center[1]
        ec.pose.position.z = 0.003
        ec.pose.orientation.w = 1.0
        ec.scale.x = self.entry_check_radius * 2
        ec.scale.y = self.entry_check_radius * 2
        ec.scale.z = 0.006
        ec.color.r = 1.0
        ec.color.g = 1.0
        ec.color.b = 0.0
        ec.color.a = 0.1
        markers.markers.append(ec)

        # 삭제
        for i in range(len(self.active_collision_zones), 10):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "collision_zone"
            m.id = i
            m.action = Marker.DELETE
            markers.markers.append(m)

        self.zone_marker_pub.publish(markers)

    def log_status(self):
        status = ""
        for cav_id in self.cav_ids:
            if self.cav_stop_flags[cav_id]:
                flag = "⛔"
            elif self.cav_slow_flags[cav_id]:
                flag = "🟡"
            else:
                flag = "✅"
            vel = self.cav_current_velocity.get(cav_id, 0)
            pri = "H" if cav_id in self.high_priority_cavs else "L"
            status += f"C{cav_id}[{pri}]:{flag}({vel:.1f}) "
        # self.get_logger().info(f"Status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
