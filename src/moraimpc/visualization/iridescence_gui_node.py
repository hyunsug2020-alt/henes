#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq 5 - Iridescence 통합 시각화 GUI
=============================================
라이브러리: pyridescence (https://github.com/koide3/iridescence)

3D 뷰 (Iridescence):
  - 차량 위치 + 방향 화살표
  - GPS 이동 궤적
  - LTV-MPC 예측 경로
  - Hybrid A* 주차 경로
  - 주차 목표 마커

ImGui 패널:
  - IMU (roll/pitch/yaw, gyro, accel)
  - GPS (fix type, 정확도, 위성 수)
  - ESKF 헤딩 (융합 결과 + 불확도)
  - 차량 상태 (속도, 조향, 모드)
  - 주차 진행상황

구독 토픽:
  /handsfree/imu, /gnss_rover/fix, /odometry/filtered
  /eskf/heading_deg, /eskf/covariance_trace
  /Ego_pose, /parking_state, /parking_path
  /local_path, /mpc_predicted_path
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from copy import copy

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String, Bool

# Iridescence
try:
    import pyridescence as rid
    from pyridescence import guik, glk
    import imgui
    _HAS_IRIDESCENCE = True
except ImportError:
    _HAS_IRIDESCENCE = False
    import warnings
    warnings.warn("pyridescence 없음 - GUI 비활성화")


# ============================================================
# 스냅샷 (스레드 안전)
# ============================================================
@dataclass
class GuiSnapshot:
    # IMU
    imu_stamp:   float = 0.0
    roll_deg:    float = 0.0
    pitch_deg:   float = 0.0
    imu_yaw_deg: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # GPS
    gps_stamp:  float = 0.0
    gps_lat:    float = 0.0
    gps_lon:    float = 0.0
    gps_h_acc:  float = 99.0
    gps_status: int   = -1

    # ESKF
    eskf_heading:  float = 0.0
    eskf_cov:      float = 99.0
    eskf_stamp:    float = 0.0

    # Odometry / Ego
    odom_stamp: float = 0.0
    odom_x:     float = 0.0
    odom_y:     float = 0.0
    odom_yaw:   float = 0.0
    odom_vx:    float = 0.0
    odom_vy:    float = 0.0

    # 주차
    park_state: str   = 'FORWARD'
    park_progress: float = 0.0

    # 경로
    local_path:   List[Tuple[float,float]] = field(default_factory=list)
    parking_path: List[Tuple[float,float]] = field(default_factory=list)
    pred_path:    List[Tuple[float,float]] = field(default_factory=list)
    gps_trail:    List[Tuple[float,float]] = field(default_factory=list)


# ============================================================
# ROS2 노드
# ============================================================
class IridescenceGuiNode(Node):

    STALE_SEC = 1.5
    MAX_TRAIL = 500

    def __init__(self) -> None:
        super().__init__('iridescence_gui_node')

        self._snap = GuiSnapshot()
        self._lock = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Imu,        '/handsfree/imu',         self._imu_cb,      sensor_qos)
        self.create_subscription(NavSatFix,  '/gnss_rover/fix',        self._gps_cb,      sensor_qos)
        self.create_subscription(Odometry,   '/odometry/filtered',     self._odom_cb,     sensor_qos)
        self.create_subscription(PoseStamped,'/Ego_pose',              self._ego_cb,      sensor_qos)
        self.create_subscription(Float64,    '/eskf/heading_deg',      self._eskf_hdg_cb, 10)
        self.create_subscription(Float64,    '/eskf/covariance_trace', self._eskf_cov_cb, 10)
        self.create_subscription(String,     '/parking_state',         self._state_cb,    10)
        self.create_subscription(Path,       '/local_path',            self._local_cb,    10)
        self.create_subscription(Path,       '/parking_path',          self._park_path_cb,10)
        self.create_subscription(Path,       '/mpc_predicted_path',    self._pred_cb,     10)

        self.get_logger().info('Iridescence GUI 노드 시작')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _quat_euler(self, x, y, z, w):
        sinr = 2*(w*x+y*z); cosr = 1-2*(x*x+y*y)
        roll = math.atan2(sinr, cosr)
        sinp = 2*(w*y-z*x)
        pitch = math.asin(max(-1., min(1., sinp)))
        siny = 2*(w*z+x*y); cosy = 1-2*(y*y+z*z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    def _imu_cb(self, msg: Imu) -> None:
        r, p, y = self._quat_euler(msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w)
        with self._lock:
            s = self._snap
            s.imu_stamp   = self._now()
            s.roll_deg    = math.degrees(r)
            s.pitch_deg   = math.degrees(p)
            s.imu_yaw_deg = math.degrees(y)
            s.gyro_x = msg.angular_velocity.x
            s.gyro_y = msg.angular_velocity.y
            s.gyro_z = msg.angular_velocity.z
            s.accel_x = msg.linear_acceleration.x
            s.accel_y = msg.linear_acceleration.y
            s.accel_z = msg.linear_acceleration.z

    def _gps_cb(self, msg: NavSatFix) -> None:
        h_acc = 99.0
        if len(msg.position_covariance) >= 1 and msg.position_covariance[0] > 0:
            h_acc = math.sqrt(msg.position_covariance[0])
        with self._lock:
            s = self._snap
            s.gps_stamp  = self._now()
            s.gps_lat    = msg.latitude
            s.gps_lon    = msg.longitude
            s.gps_h_acc  = h_acc
            s.gps_status = msg.status.status

    def _odom_cb(self, msg: Odometry) -> None:
        r, p, y = self._quat_euler(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        with self._lock:
            s = self._snap
            s.odom_stamp = self._now()
            s.odom_x  = msg.pose.pose.position.x
            s.odom_y  = msg.pose.pose.position.y
            s.odom_yaw= y
            s.odom_vx = msg.twist.twist.linear.x
            s.odom_vy = msg.twist.twist.linear.y
            # GPS 궤적 추가
            if len(s.gps_trail) == 0 or math.hypot(
                    s.odom_x - s.gps_trail[-1][0],
                    s.odom_y - s.gps_trail[-1][1]) > 0.5:
                s.gps_trail.append((s.odom_x, s.odom_y))
                if len(s.gps_trail) > self.MAX_TRAIL:
                    s.gps_trail.pop(0)

    def _ego_cb(self, msg: PoseStamped) -> None:
        r, p, y = self._quat_euler(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w)
        with self._lock:
            s = self._snap
            s.odom_x   = msg.pose.position.x
            s.odom_y   = msg.pose.position.y
            s.odom_yaw = y

    def _eskf_hdg_cb(self, msg: Float64) -> None:
        with self._lock:
            self._snap.eskf_stamp   = self._now()
            self._snap.eskf_heading = float(msg.data)

    def _eskf_cov_cb(self, msg: Float64) -> None:
        with self._lock:
            self._snap.eskf_cov = float(msg.data)

    def _state_cb(self, msg: String) -> None:
        with self._lock:
            self._snap.park_state = msg.data

    def _path_to_xy(self, path: Path) -> List[Tuple[float,float]]:
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    def _local_cb(self, msg: Path) -> None:
        with self._lock:
            self._snap.local_path = self._path_to_xy(msg)

    def _park_path_cb(self, msg: Path) -> None:
        with self._lock:
            self._snap.parking_path = self._path_to_xy(msg)

    def _pred_cb(self, msg: Path) -> None:
        with self._lock:
            self._snap.pred_path = self._path_to_xy(msg)

    def get_snapshot(self) -> GuiSnapshot:
        with self._lock:
            return copy(self._snap)

    def fresh(self, stamp: float) -> bool:
        now = self._now()
        return stamp > 0.0 and (now - stamp) <= self.STALE_SEC


# ============================================================
# Iridescence GUI 루프
# ============================================================

def _run_gui(node: IridescenceGuiNode) -> None:
    """Iridescence 뷰어 루프 (메인 스레드)"""
    if not _HAS_IRIDESCENCE:
        print("[WARN] pyridescence 없음 - GUI 시작 불가")
        return

    viewer = guik.LightViewer.instance(
        resolution=[1600, 900],
        title='MORAI MPC Monitor - Ioniq 5')
    viewer.enable_vsync()

    # 좌표축
    viewer.update_drawable(
        'world_axis',
        glk.primitives.coordinate_system(),
        guik.VertexColor(np.eye(4, dtype=np.float32)))

    while viewer.spin_once():
        s = node.get_snapshot()
        now = node.get_clock().now().nanoseconds * 1e-9

        # ── 3D: 차량 위치 (화살표) ──
        yaw = s.odom_yaw
        T = np.eye(4, dtype=np.float32)
        cy, sy = math.cos(yaw), math.sin(yaw)
        T[0,0]=cy; T[0,1]=-sy; T[1,0]=sy; T[1,1]=cy
        T[0,3]=s.odom_x; T[1,3]=s.odom_y; T[2,3]=0.5
        viewer.update_drawable(
            'vehicle',
            glk.primitives.cone(),
            guik.FlatColor(0.2, 0.6, 1.0, 1.0, T))

        # ── 3D: GPS 궤적 ──
        if len(s.gps_trail) >= 2:
            pts = np.array([[x, y, 0.1] for x,y in s.gps_trail], dtype=np.float32)
            viewer.update_drawable(
                'gps_trail',
                glk.ThinLines(pts),
                guik.FlatColor(0.0, 1.0, 0.4, 0.8))

        # ── 3D: 전진 경로 ──
        if len(s.local_path) >= 2:
            pts = np.array([[x, y, 0.2] for x,y in s.local_path], dtype=np.float32)
            viewer.update_drawable(
                'local_path',
                glk.ThinLines(pts),
                guik.FlatColor(1.0, 0.8, 0.0, 0.9))

        # ── 3D: 주차 경로 (Hybrid A*) ──
        if len(s.parking_path) >= 2:
            pts = np.array([[x, y, 0.3] for x,y in s.parking_path], dtype=np.float32)
            viewer.update_drawable(
                'parking_path',
                glk.ThinLines(pts),
                guik.FlatColor(1.0, 0.3, 0.3, 1.0))

        # ── 3D: MPC 예측 궤적 ──
        if len(s.pred_path) >= 2:
            pts = np.array([[x, y, 0.4] for x,y in s.pred_path], dtype=np.float32)
            viewer.update_drawable(
                'pred_path',
                glk.ThinLines(pts),
                guik.FlatColor(0.3, 0.7, 1.0, 0.7))

        # ── ImGui UI 패널 ──
        _draw_imgui(s, node)


def _draw_imgui(s: GuiSnapshot, node: IridescenceGuiNode) -> None:
    """ImGui 패널 렌더링"""
    now = node.get_clock().now().nanoseconds * 1e-9

    # ── 차량 상태 패널 ──
    imgui.set_next_window_pos((10, 10), imgui.ONCE)
    imgui.set_next_window_size((320, 300), imgui.ONCE)
    imgui.begin('Ioniq 5 상태')

    # 주행 모드
    mode_col = {
        'FORWARD':  (0.2, 0.8, 0.2, 1.0),
        'REVERSE':  (1.0, 0.5, 0.0, 1.0),
        'PARKING':  (1.0, 0.3, 0.3, 1.0),
        'APPROACH': (1.0, 0.8, 0.0, 1.0),
        'PLAN':     (0.5, 0.5, 1.0, 1.0),
        'DONE':     (0.2, 1.0, 0.2, 1.0),
    }.get(s.park_state, (0.6, 0.6, 0.6, 1.0))
    imgui.text_colored(f'모드: {s.park_state}', *mode_col)

    imgui.separator()
    speed = math.hypot(s.odom_vx, s.odom_vy)
    imgui.text(f'속도:    {speed:+.2f} m/s  ({speed*3.6:.1f} km/h)')
    imgui.text(f'위치:    ({s.odom_x:.2f}, {s.odom_y:.2f}) m')
    imgui.text(f'방향:    {math.degrees(s.odom_yaw):.1f}°')

    imgui.separator()
    imgui.text('ESKF 헤딩')
    eskf_fresh = node.fresh(s.eskf_stamp)
    col = (0.2, 0.9, 0.2, 1.0) if eskf_fresh else (0.8, 0.3, 0.3, 1.0)
    imgui.text_colored(f'  융합 헤딩:  {s.eskf_heading:.2f}°', *col)
    if s.eskf_cov > 0:
        sigma = math.degrees(math.sqrt(s.eskf_cov / 15.0 + 1e-10))
        imgui.text(f'  불확도:     ±{sigma:.2f}°')

    imgui.end()

    # ── IMU 패널 ──
    imgui.set_next_window_pos((10, 320), imgui.ONCE)
    imgui.set_next_window_size((320, 240), imgui.ONCE)
    imgui.begin('IMU')

    imu_fresh = node.fresh(s.imu_stamp)
    status_col = (0.2, 0.9, 0.2, 1.0) if imu_fresh else (0.8, 0.3, 0.3, 1.0)
    imgui.text_colored('● LIVE' if imu_fresh else '○ STALE', *status_col)

    imgui.text(f'Roll:   {s.roll_deg:+7.2f}°')
    imgui.text(f'Pitch:  {s.pitch_deg:+7.2f}°')
    imgui.text(f'Yaw:    {s.imu_yaw_deg:+7.2f}°')
    imgui.separator()
    imgui.text(f'Gyro Z: {s.gyro_z:+.3f} rad/s')
    imgui.text(f'Accel X:{s.accel_x:+.3f} m/s²')
    imgui.text(f'Accel Y:{s.accel_y:+.3f} m/s²')
    imgui.text(f'Accel Z:{s.accel_z:+.3f} m/s²')

    imgui.end()

    # ── GPS 패널 ──
    imgui.set_next_window_pos((340, 10), imgui.ONCE)
    imgui.set_next_window_size((300, 200), imgui.ONCE)
    imgui.begin('GPS')

    gps_fresh = node.fresh(s.gps_stamp)
    gps_col = (0.2, 0.9, 0.2, 1.0) if gps_fresh else (0.8, 0.3, 0.3, 1.0)
    imgui.text_colored('● LIVE' if gps_fresh else '○ STALE', *gps_col)

    fix_str = {-1: 'NO FIX', 0: 'FIX', 1: 'SBAS', 2: 'GBAS'}.get(
        s.gps_status, 'UNKNOWN')
    fix_col = (0.2, 0.9, 0.2, 1.0) if s.gps_status >= 0 else (0.9, 0.3, 0.3, 1.0)
    imgui.text_colored(f'Fix: {fix_str}', *fix_col)

    acc_col = (0.2, 0.9, 0.2, 1.0) if s.gps_h_acc < 0.1 else \
              (1.0, 0.8, 0.0, 1.0) if s.gps_h_acc < 0.5 else \
              (0.9, 0.3, 0.3, 1.0)
    imgui.text_colored(f'수평 정확도: {s.gps_h_acc:.3f} m', *acc_col)
    imgui.text(f'위도: {s.gps_lat:.7f}°')
    imgui.text(f'경도: {s.gps_lon:.7f}°')

    imgui.end()

    # ── 주차 진행 패널 ──
    if s.park_state in ('APPROACH', 'PLAN', 'REVERSE', 'FINE_TUNE', 'DONE'):
        imgui.set_next_window_pos((340, 220), imgui.ONCE)
        imgui.set_next_window_size((300, 150), imgui.ONCE)
        imgui.begin('주차 상황')
        imgui.text(f'상태: {s.park_state}')
        imgui.text(f'경로 웨이포인트: {len(s.parking_path)}개')
        if s.park_state == 'DONE':
            imgui.text_colored('✓ 주차 완료!', 0.2, 1.0, 0.2, 1.0)
        imgui.end()


# ============================================================
# main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = IridescenceGuiNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if _HAS_IRIDESCENCE:
        try:
            _run_gui(node)
        except Exception as e:
            node.get_logger().error(f'GUI 오류: {e}')
    else:
        node.get_logger().warn('pyridescence 없음 - ROS만 실행')
        spin_thread.join()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
