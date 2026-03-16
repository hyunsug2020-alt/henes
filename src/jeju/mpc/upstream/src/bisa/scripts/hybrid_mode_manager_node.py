#!/usr/bin/env python3
"""
하이브리드 모드 매니저 노드
============================
드라이브 모드를 관리하고 주차 시 Hybrid A* + RTI-NMPC를 조율합니다.

상태 머신:
  IDLE      → 아무것도 하지 않음
  FORWARD   → LTV-MPC 경로 추종 (전진)
  REVERSE   → RTI-NMPC 경로 추종 (후진)
  PARKING   → 주차구역 감지 후 Hybrid A* 경로 계획 + RTI-NMPC 추종

전환 조건:
  FORWARD → PARKING : /parking_spot 수신 + 근거리 (5m 이내)
  PARKING → FORWARD : 주차 완료 또는 취소
  FORWARD → REVERSE : 외부 명령 (/drive_mode_cmd 토픽)
  REVERSE → FORWARD : 외부 명령

토픽:
  구독:
    /parking_spot        (bisa/ParkingSpot)   카메라 감지 결과
    /odometry/filtered   (nav_msgs/Odometry)  차량 자세/속도
    /Ego_pose            (geometry_msgs/PoseStamped)  차량 자세
    /drive_mode_cmd      (std_msgs/String)    외부 모드 전환 명령
    /local_path          (nav_msgs/Path)      전진 경로 (LTV-MPC용)

  발행:
    /drive_mode          (bisa/DriveMode)     현재 모드 정보
    /parking_path        (nav_msgs/Path)      주차 경로 (RTI-NMPC용)
    /cmd_vel_mux         (geometry_msgs/Twist) 게이트 출력 (제어 명령 선택)
    /parking_marker      (visualization_msgs/Marker) 시각화

의존성:
  - hybrid_astar.py (같은 scripts/ 폴더)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import math
import time
import sys
import os
from enum import Enum, auto
from typing import Optional, List, Tuple

import numpy as np

# ROS 메시지
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import (PoseStamped, Pose, Twist,
                                Point, Quaternion)
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# bisa 커스텀 메시지 (빌드 후 사용 가능)
try:
    from bisa.msg import ParkingSpot, DriveMode
    _has_custom_msgs = True
except ImportError:
    _has_custom_msgs = False

# Hybrid A* (같은 디렉토리)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hybrid_astar import (HybridAStarPlanner, HybridAStarConfig,
                           Obstacle, HybridAStarPlanner)


# ============================================================
# 상태 머신 열거형
# ============================================================
class DriveState(Enum):
    IDLE    = auto()
    FORWARD = auto()
    REVERSE = auto()
    PARKING_PLAN   = auto()   # 경로 계획 중
    PARKING_TRACK  = auto()   # 주차 추종 중
    PARKING_DONE   = auto()   # 주차 완료


STATE_TO_STR = {
    DriveState.IDLE:           'IDLE',
    DriveState.FORWARD:        'FORWARD',
    DriveState.REVERSE:        'REVERSE',
    DriveState.PARKING_PLAN:   'PARKING',
    DriveState.PARKING_TRACK:  'PARKING',
    DriveState.PARKING_DONE:   'IDLE',
}


# ============================================================
# 하이브리드 모드 매니저
# ============================================================
class HybridModeManager(Node):

    def __init__(self):
        super().__init__('hybrid_mode_manager')

        # ---- 파라미터 선언 ----
        self.declare_parameter('initial_mode', 'FORWARD')
        self.declare_parameter('parking_trigger_distance', 5.0)  # [m]
        self.declare_parameter('parking_done_distance', 0.2)      # [m]
        self.declare_parameter('parking_done_angle_deg', 10.0)    # [deg]
        self.declare_parameter('parking_speed', 0.3)              # [m/s]
        self.declare_parameter('parking_timeout_sec', 60.0)       # [s]
        self.declare_parameter('spot_confidence_threshold', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Hybrid A* 파라미터
        self.declare_parameter('ha_xy_resolution',    0.10)
        self.declare_parameter('ha_theta_deg',        5.0)
        self.declare_parameter('ha_wheelbase',        0.30)
        self.declare_parameter('ha_max_steer_deg',   35.0)
        self.declare_parameter('ha_vehicle_length',   0.45)
        self.declare_parameter('ha_vehicle_width',    0.25)
        self.declare_parameter('ha_use_reed_shepp',   True)

        # 파라미터 읽기
        self._read_params()

        # ---- 내부 상태 ----
        self.state = DriveState.IDLE
        self.current_pose: Optional[Pose] = None
        self.current_vel_x = 0.0
        self.current_vel_yaw = 0.0
        self.target_spot: Optional[object] = None   # ParkingSpot
        self.parking_path: List[Tuple] = []
        self.parking_start_time: Optional[float] = None
        self.forward_path: Optional[Path] = None

        # Hybrid A* 계획기
        self.ha_planner = HybridAStarPlanner(self.ha_cfg)

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1)

        # ---- 구독자 ----
        if _has_custom_msgs:
            self.spot_sub = self.create_subscription(
                ParkingSpot, '/parking_spot', self._spot_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/Ego_pose', self._pose_callback, sensor_qos)
        self.mode_cmd_sub = self.create_subscription(
            String, '/drive_mode_cmd', self._mode_cmd_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/local_path', self._forward_path_callback, 10)

        # ---- 발행자 ----
        self.parking_path_pub = self.create_publisher(Path, '/parking_path', 10)
        if _has_custom_msgs:
            self.mode_pub = self.create_publisher(DriveMode, '/drive_mode', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/parking_marker', 10)

        # ---- 제어 루프 타이머 ----
        period_ms = int(1000.0 / self.publish_rate_hz)
        self.timer = self.create_wall_timer(
            rclpy.duration.Duration(nanoseconds=period_ms * 1_000_000).nanoseconds / 1e9,
            # rclpy timer는 seconds 단위
            self._control_loop)
        # 위 방법 대신 간단히:
        self.timer.cancel()
        self.timer = self.create_wall_timer(
            period_ms / 1000.0,
            self._control_loop)

        # 초기 모드 설정
        init_mode = self.get_parameter('initial_mode').value
        self._set_mode_from_str(init_mode)

        self.get_logger().info('하이브리드 모드 매니저 시작')
        self.get_logger().info(f'  초기 모드: {init_mode}')
        self.get_logger().info(f'  주차 트리거 거리: {self.parking_trigger_dist:.1f}m')
        self.get_logger().info(f'  Hybrid A* 해상도: {self.ha_cfg.xy_resolution:.2f}m')

    def _read_params(self):
        """파라미터 로드"""
        self.parking_trigger_dist = self.get_parameter(
            'parking_trigger_distance').value
        self.parking_done_dist    = self.get_parameter(
            'parking_done_distance').value
        self.parking_done_angle   = math.radians(
            self.get_parameter('parking_done_angle_deg').value)
        self.parking_speed        = self.get_parameter('parking_speed').value
        self.parking_timeout      = self.get_parameter('parking_timeout_sec').value
        self.spot_conf_thresh     = self.get_parameter(
            'spot_confidence_threshold').value
        self.publish_rate_hz      = self.get_parameter('publish_rate_hz').value

        # Hybrid A* 설정
        self.ha_cfg = HybridAStarConfig(
            xy_resolution   = self.get_parameter('ha_xy_resolution').value,
            theta_resolution= math.radians(self.get_parameter('ha_theta_deg').value),
            wheelbase       = self.get_parameter('ha_wheelbase').value,
            max_steer       = math.radians(self.get_parameter('ha_max_steer_deg').value),
            vehicle_length  = self.get_parameter('ha_vehicle_length').value,
            vehicle_width   = self.get_parameter('ha_vehicle_width').value,
            use_reed_shepp  = self.get_parameter('ha_use_reed_shepp').value,
        )

    # =========================================================
    # 콜백 함수들
    # =========================================================

    def _odom_callback(self, msg: Odometry):
        """오도메트리 수신 → 속도 업데이트"""
        self.current_vel_x   = msg.twist.twist.linear.x
        self.current_vel_yaw = msg.twist.twist.angular.z
        if self.current_pose is None:
            self.current_pose = msg.pose.pose

    def _pose_callback(self, msg: PoseStamped):
        """차량 자세 수신"""
        self.current_pose = msg.pose

    def _forward_path_callback(self, msg: Path):
        """전진 경로 수신 (LTV-MPC용 경로)"""
        self.forward_path = msg

    def _mode_cmd_callback(self, msg: String):
        """외부 모드 전환 명령"""
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'모드 전환 명령 수신: {cmd}')
        self._set_mode_from_str(cmd)

    def _spot_callback(self, msg):
        """주차구역 감지 콜백 (ParkingSpot)"""
        if msg.confidence < self.spot_conf_thresh:
            return

        if self.state not in (DriveState.FORWARD, DriveState.IDLE):
            return

        if self.current_pose is None:
            return

        # 감지 거리 계산
        dx = msg.spot_pose.position.x - self.current_pose.position.x
        dy = msg.spot_pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info(
            f'주차구역 감지: ID={msg.spot_id}, 거리={dist:.2f}m, '
            f'신뢰도={msg.confidence:.2f}')

        if dist <= self.parking_trigger_dist:
            self.target_spot = msg
            self._transition_to_parking()

    # =========================================================
    # 상태 전환
    # =========================================================

    def _set_mode_from_str(self, mode_str: str):
        """문자열로 모드 전환"""
        if mode_str == 'FORWARD':
            self.state = DriveState.FORWARD
        elif mode_str == 'REVERSE':
            self.state = DriveState.REVERSE
        elif mode_str == 'PARKING':
            self._transition_to_parking()
        elif mode_str == 'IDLE':
            self.state = DriveState.IDLE
        self._publish_mode(reason=f'외부 명령: {mode_str}')

    def _transition_to_parking(self):
        """주차 모드 전환: 경로 계획 시작"""
        if self.target_spot is None and self.current_pose is not None:
            self.get_logger().warn('주차 목표 없음, 전환 취소')
            return

        self.get_logger().info('주차 모드 전환: Hybrid A* 경로 계획 시작')
        self.state = DriveState.PARKING_PLAN
        self.parking_start_time = time.time()
        self._publish_mode(reason='주차구역 감지')

        # 별도 스레드에서 계획 (ROS2 타이머 블로킹 방지)
        # 간단히는 다음 control_loop에서 처리
        self._run_hybrid_astar_planning()

    def _run_hybrid_astar_planning(self):
        """Hybrid A* 경로 계획 실행"""
        if self.current_pose is None:
            self.get_logger().error('현재 자세 없음, 경로 계획 실패')
            self.state = DriveState.FORWARD
            return

        # 시작점
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_theta = self._pose_yaw(self.current_pose)

        # 목표점 (주차구역 중심)
        if self.target_spot is not None:
            goal_x = self.target_spot.spot_pose.position.x
            goal_y = self.target_spot.spot_pose.position.y
            goal_theta = self._quat_to_yaw(
                self.target_spot.spot_pose.orientation)
        else:
            self.get_logger().warn('목표 주차구역 없음, 정면 5m로 설정')
            c = math.cos(start_theta)
            s = math.sin(start_theta)
            goal_x = start_x + 3.0 * c
            goal_y = start_y + 3.0 * s
            goal_theta = start_theta

        self.get_logger().info(
            f'Hybrid A* 계획: '
            f'시작=({start_x:.2f},{start_y:.2f},{math.degrees(start_theta):.1f}°) '
            f'목표=({goal_x:.2f},{goal_y:.2f},{math.degrees(goal_theta):.1f}°)')

        # 맵 범위 설정 (시작-목표 주변)
        margin = 3.0
        map_orig_x = min(start_x, goal_x) - margin
        map_orig_y = min(start_y, goal_y) - margin
        map_size_x = abs(goal_x - start_x) + 2 * margin
        map_size_y = abs(goal_y - start_y) + 2 * margin

        # 경로 계획 (장애물 없을 때도 동작)
        t_start = time.time()
        raw_path = self.ha_planner.plan(
            start_x=start_x, start_y=start_y, start_theta=start_theta,
            goal_x=goal_x,   goal_y=goal_y,   goal_theta=goal_theta,
            map_origin_x=map_orig_x, map_origin_y=map_orig_y
        )
        t_elapsed = time.time() - t_start

        if raw_path is None or len(raw_path) < 2:
            self.get_logger().warn(f'Hybrid A* 경로 없음 ({t_elapsed*1000:.0f}ms), '
                                    '직선 경로로 대체')
            raw_path = self._generate_straight_path(
                start_x, start_y, start_theta,
                goal_x, goal_y, goal_theta)

        self.get_logger().info(
            f'Hybrid A* 완료: {len(raw_path)}점, {t_elapsed*1000:.0f}ms')

        # 경로 평활화
        smooth_path = HybridAStarPlanner.smooth_path(raw_path)

        # 등간격 웨이포인트 생성
        waypoints = HybridAStarPlanner.path_to_ros_waypoints(
            smooth_path, ds=0.05)

        self.parking_path = waypoints
        self._publish_parking_path(waypoints)

        # 추종 모드로 전환
        self.state = DriveState.PARKING_TRACK
        self._publish_mode(reason='주차 경로 계획 완료')
        self.get_logger().info('주차 경로 계획 완료, RTI-NMPC 추종 시작')

    # =========================================================
    # 제어 루프
    # =========================================================

    def _control_loop(self):
        """메인 제어 루프 (20Hz)"""
        if self.current_pose is None:
            return

        if self.state == DriveState.FORWARD:
            self._publish_mode(reason='전진 추종')

        elif self.state == DriveState.REVERSE:
            self._publish_mode(reason='후진 추종')

        elif self.state == DriveState.PARKING_TRACK:
            self._check_parking_completion()

        elif self.state == DriveState.PARKING_DONE:
            self.get_logger().info_once('주차 완료!')
            self._publish_mode(reason='주차 완료')
            time.sleep(2.0)
            self.state = DriveState.IDLE

        # 시각화 발행
        self._publish_visualization()

    def _check_parking_completion(self):
        """주차 완료 조건 확인"""
        if self.target_spot is None or self.current_pose is None:
            return

        # 시간초과 확인
        if self.parking_start_time is not None:
            elapsed = time.time() - self.parking_start_time
            if elapsed > self.parking_timeout:
                self.get_logger().warn(f'주차 타임아웃 ({elapsed:.0f}s), 전진 복귀')
                self.state = DriveState.FORWARD
                self._publish_mode(reason='주차 타임아웃')
                return

        # 목표 거리 확인
        dx = self.target_spot.spot_pose.position.x - self.current_pose.position.x
        dy = self.target_spot.spot_pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        # 방향 오차 확인
        goal_yaw = self._quat_to_yaw(self.target_spot.spot_pose.orientation)
        cur_yaw  = self._pose_yaw(self.current_pose)
        d_theta  = abs(math.atan2(math.sin(goal_yaw - cur_yaw),
                                   math.cos(goal_yaw - cur_yaw)))

        if dist < self.parking_done_dist and d_theta < self.parking_done_angle:
            self.get_logger().info(
                f'주차 완료! 거리={dist*100:.0f}cm, 방향오차={math.degrees(d_theta):.1f}°')
            self.state = DriveState.PARKING_DONE
            self._publish_mode(reason='주차 완료 감지')

    # =========================================================
    # 발행 함수들
    # =========================================================

    def _publish_mode(self, reason: str = ''):
        """현재 드라이브 모드 발행"""
        if not _has_custom_msgs:
            return
        msg = DriveMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.mode   = STATE_TO_STR.get(self.state, 'IDLE')
        msg.reason = reason
        msg.target_spot_id = (self.target_spot.spot_id
                               if self.target_spot else -1)
        msg.path_planned = (self.state == DriveState.PARKING_TRACK and
                             len(self.parking_path) > 0)

        # 주차 진행률
        if (self.state == DriveState.PARKING_TRACK and
                self.target_spot is not None and
                self.current_pose is not None):
            dx = self.target_spot.spot_pose.position.x - self.current_pose.position.x
            dy = self.target_spot.spot_pose.position.y - self.current_pose.position.y
            rem = math.sqrt(dx*dx + dy*dy)
            total = self.parking_trigger_dist
            msg.parking_progress = max(0.0, min(1.0, 1.0 - rem / total))
        else:
            msg.parking_progress = 0.0

        self.mode_pub.publish(msg)

    def _publish_parking_path(self, waypoints: List[Tuple]):
        """주차 경로 발행 (nav_msgs/Path)"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y, theta in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.z = math.sin(theta / 2)
            ps.pose.orientation.w = math.cos(theta / 2)
            path_msg.poses.append(ps)

        self.parking_path_pub.publish(path_msg)
        self.get_logger().info(
            f'주차 경로 발행: {len(waypoints)}개 웨이포인트 → /parking_path')

    def _publish_visualization(self):
        """시각화 마커 발행"""
        markers = MarkerArray()

        # 주차구역 마커
        if self.target_spot is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'parking_spot'
            m.id = 0
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = self.target_spot.spot_pose
            m.scale.x = self.target_spot.length
            m.scale.y = self.target_spot.width
            m.scale.z = 0.05
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.5
            markers.markers.append(m)

        # 주차 경로 마커
        if self.parking_path:
            m2 = Marker()
            m2.header.frame_id = 'map'
            m2.header.stamp = self.get_clock().now().to_msg()
            m2.ns = 'parking_path'
            m2.id = 1
            m2.type = Marker.LINE_STRIP
            m2.action = Marker.ADD
            m2.scale.x = 0.03
            m2.color.r = 0.0
            m2.color.g = 0.5
            m2.color.b = 1.0
            m2.color.a = 1.0
            for x, y, _ in self.parking_path:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.05
                m2.points.append(p)
            markers.markers.append(m2)

        self.marker_pub.publish(markers)

    # =========================================================
    # 유틸리티
    # =========================================================

    def _pose_yaw(self, pose: Pose) -> float:
        """Pose에서 yaw 추출"""
        q = pose.orientation
        n = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if n > 1e-6:
            qz, qw = q.z/n, q.w/n
            return math.atan2(2*qw*qz, 1 - 2*qz*qz)
        return 0.0

    def _quat_to_yaw(self, q) -> float:
        """Quaternion → yaw"""
        n = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if n > 1e-6:
            qz, qw = q.z/n, q.w/n
            return math.atan2(2*qw*qz, 1 - 2*qz*qz)
        return 0.0

    def _generate_straight_path(self,
                                 sx, sy, sth,
                                 gx, gy, gth,
                                 ds: float = 0.05
                                 ) -> List[Tuple]:
        """Hybrid A* 실패 시 직선 경로 대체"""
        dist = math.sqrt((gx-sx)**2 + (gy-sy)**2)
        n = max(int(dist / ds), 2)
        path = []
        for i in range(n + 1):
            t = i / n
            x = sx + t * (gx - sx)
            y = sy + t * (gy - sy)
            theta = sth + t * (gth - sth)
            path.append((x, y, theta, 1))  # direction=1 (전진)
        return path


# ============================================================
# 메인
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = HybridModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
