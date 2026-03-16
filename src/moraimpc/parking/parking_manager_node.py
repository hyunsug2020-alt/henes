#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq 5 주차 매니저 노드
================================
논문 기반 통합 주차 시스템:
  1. [2406.06427v3.pdf]        ESKF - GPS+IMU 정밀 헤딩
  2. [Linear_Time-Varying_MPC] LTV-MPC - 전진 경로 추종
  3. [2410.12170v1.pdf]        RTI-NMPC - 후진 주차 추종
  4. [하이브리드의 근본 논문]    Hybrid A* - 주차 경로 계획
  5. [하이브리드의 구조적 설계]  Guided Hybrid A* - 비지빌리티 다이어그램
  6. [하이브리드와 mpc의 연결]  계층 구조: Hybrid A* → RTI-NMPC

주차 상태 머신:
  FORWARD    : LTV-MPC로 전진 주행
  APPROACH   : 주차구역 접근 (속도 감속)
  PLAN       : Hybrid A* 경로 계획
  REVERSE    : RTI-NMPC 후진 주차
  FINE_TUNE  : 최종 위치 미세 조정
  DONE       : 주차 완료

차량: Ioniq 5 SUV (wheelbase=3.0m)
"""

from __future__ import annotations

import math
import time
import sys
import os
from enum import Enum, auto
from typing import Optional, List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import String, Float64, Bool
from visualization_msgs.msg import Marker, MarkerArray

# Hybrid A* (같은 패키지)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'navigation'))

try:
    from hybrid_astar import (HybridAStarPlanner, HybridAStarConfig, Obstacle)
    _HAS_HASTAR = True
except ImportError:
    _HAS_HASTAR = False

# MORAI 메시지
try:
    from morai_msgs.msg import EgoVehicleStatus
    _HAS_MORAI = True
except ImportError:
    _HAS_MORAI = False


# ============================================================
# 상태 머신
# ============================================================
class ParkState(Enum):
    FORWARD   = auto()   # LTV-MPC 전진
    APPROACH  = auto()   # 주차구역 접근
    PLAN      = auto()   # Hybrid A* 경로 계획
    REVERSE   = auto()   # RTI-NMPC 후진
    FINE_TUNE = auto()   # 미세 조정
    DONE      = auto()   # 완료
    IDLE      = auto()   # 대기


STATE_NAMES = {
    ParkState.FORWARD:   'FORWARD',
    ParkState.APPROACH:  'APPROACH',
    ParkState.PLAN:      'PLAN',
    ParkState.REVERSE:   'REVERSE',
    ParkState.FINE_TUNE: 'FINE_TUNE',
    ParkState.DONE:      'DONE',
    ParkState.IDLE:      'IDLE',
}


# ============================================================
# Ioniq 5 RTI-NMPC (Python, 논문 2410.12170v1.pdf 기반)
# ============================================================
class RTINMPCIoniq5:
    """
    RTI-NMPC 후진 주차 컨트롤러 (Ioniq 5 파라미터)
    상태: x = [px, py, ψ, v, κ]^T
    입력: u = [av, a_κ]^T
    암시적 오일러 이산화 + OSQP QP
    """

    NX = 5
    NU = 2

    def __init__(self):
        # Ioniq 5 파라미터
        self.N  = 20
        self.Ts = 0.1
        self.wheelbase = 3.0
        self.kappa_max =  math.tan(math.radians(35)) / 3.0  # 0.234
        self.kappa_min = -self.kappa_max
        self.v_max  =  0.5   # 주차 속도
        self.v_min  = -1.5   # 후진 속도

        # 비용 가중치
        self.Q = np.diag([15.0, 15.0, 8.0, 2.0, 1.0])  # 상태
        self.R = np.diag([0.5, 1.0])                     # 입력

        # 내부 상태
        self.kappa = 0.0
        self.v      = 0.0
        self.u_warm = [np.zeros(self.NU) for _ in range(self.N)]
        self.initialized = False

    def reset(self):
        self.kappa = 0.0
        self.v = 0.0
        self.u_warm = [np.zeros(self.NU) for _ in range(self.N)]
        self.initialized = False

    def _dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """운동학 f(x, u) = [v·cosψ, v·sinψ, v·κ, av, aκ]"""
        psi, v, kappa = x[2], x[3], x[4]
        return np.array([
            v * math.cos(psi),
            v * math.sin(psi),
            v * kappa,
            u[0],
            u[1],
        ])

    def _jacobian_fx(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        psi, v, kappa = x[2], x[3], x[4]
        Jx = np.zeros((self.NX, self.NX))
        Jx[0, 2] = -v * math.sin(psi)
        Jx[0, 3] =  math.cos(psi)
        Jx[1, 2] =  v * math.cos(psi)
        Jx[1, 3] =  math.sin(psi)
        Jx[2, 3] =  kappa
        Jx[2, 4] =  v
        return Jx

    def _implicit_euler(self, x_k: np.ndarray, u_k: np.ndarray) -> np.ndarray:
        """Newton 반복으로 암시적 오일러 풀이"""
        x_next = x_k.copy()
        for _ in range(3):
            r = x_next - x_k - self.Ts * self._dynamics(x_next, u_k)
            Jr = np.eye(self.NX) - self.Ts * self._jacobian_fx(x_next, u_k)
            x_next -= np.linalg.solve(Jr, r)
        x_next[2] = math.atan2(math.sin(x_next[2]), math.cos(x_next[2]))
        x_next[4] = np.clip(x_next[4], self.kappa_min, self.kappa_max)
        x_next[3] = np.clip(x_next[3], self.v_min, self.v_max)
        return x_next

    def _linearize(self, x_k, u_k):
        """이산화 + 선형화 → A_d, B_d, d_k"""
        Fx = self._jacobian_fx(x_k, u_k)
        Fu = np.zeros((self.NX, self.NU))
        Fu[3, 0] = 1.0
        Fu[4, 1] = 1.0
        M   = np.eye(self.NX) - self.Ts * Fx
        A_d = np.linalg.inv(M)
        B_d = A_d @ (self.Ts * Fu)
        x_nl = self._implicit_euler(x_k, u_k)
        d_k  = x_nl - A_d @ x_k - B_d @ u_k
        return A_d, B_d, d_k

    def compute(self, x0: np.ndarray,
                x_ref: List[np.ndarray]) -> Tuple[float, float]:
        """
        RTI-NMPC 제어 계산
        반환: (v_cmd, omega_cmd)
        """
        if not self.initialized:
            self.kappa = x0[4]
            self.v     = x0[3]
            self.initialized = True

        N = self.N

        # warm-start 궤적 시뮬레이션
        x_traj = [x0.copy()]
        for k in range(N):
            x_traj.append(self._implicit_euler(x_traj[-1], self.u_warm[k]))

        # 선형화
        A_seq, B_seq, d_seq = [], [], []
        for k in range(N):
            Ad, Bd, dk = self._linearize(x_traj[k], self.u_warm[k])
            A_seq.append(Ad); B_seq.append(Bd); d_seq.append(dk)

        # 배치 QP (간소화: 1-step SQP)
        # min Σ ||x(k)-xr(k)||_Q + ||u(k)||_R
        try:
            import osqp
            import scipy.sparse as sp

            n_dec = N * self.NU

            # 배치 예측 행렬 Gamma
            Phi   = np.zeros(((N+1)*self.NX, self.NX))
            Gamma = np.zeros(((N+1)*self.NX, n_dec))
            sigma = np.zeros((N+1)*self.NX)

            Phi[:self.NX, :] = np.eye(self.NX)
            Phi_k = np.eye(self.NX)
            sig_k = np.zeros(self.NX)

            for k in range(N):
                Phi_k  = A_seq[k] @ Phi_k
                sig_k  = A_seq[k] @ sig_k + d_seq[k]
                Phi[(k+1)*self.NX:(k+2)*self.NX, :] = Phi_k
                sigma[(k+1)*self.NX:(k+2)*self.NX]  = sig_k
                for j in range(k+1):
                    if j == k:
                        Gamma[(k+1)*self.NX:(k+2)*self.NX, j*self.NU:(j+1)*self.NU] = B_seq[k]
                    else:
                        Gamma[(k+1)*self.NX:(k+2)*self.NX, j*self.NU:(j+1)*self.NU] = \
                            A_seq[k] @ Gamma[k*self.NX:(k+1)*self.NX, j*self.NU:(j+1)*self.NU]

            # Q_bar, R_bar
            Q_bar = np.block([[self.Q if k <= N else 3*self.Q
                               for k in range(1)] for _ in range(1)])
            Q_bar = np.zeros(((N+1)*self.NX, (N+1)*self.NX))
            for k in range(N+1):
                w = 3.0 if k == N else 1.0
                Q_bar[k*self.NX:(k+1)*self.NX, k*self.NX:(k+1)*self.NX] = w * self.Q

            R_bar = np.zeros((n_dec, n_dec))
            for k in range(N):
                R_bar[k*self.NU:(k+1)*self.NU, k*self.NU:(k+1)*self.NU] = self.R

            X_ref = np.zeros((N+1)*self.NX)
            for k in range(N+1):
                idx = min(k, len(x_ref)-1)
                X_ref[k*self.NX:(k+1)*self.NX] = x_ref[idx]

            GtQ  = Gamma.T @ Q_bar
            H    = GtQ @ Gamma + R_bar
            H    = 0.5*(H+H.T) + 1e-6*np.eye(n_dec)
            e_fr = Phi @ x0 + sigma - X_ref
            f    = GtQ @ e_fr

            # 제약
            lb = np.tile([-3.0, -2.0], N)
            ub = np.tile([ 1.0,  2.0], N)
            A_c = sp.eye(n_dec, format='csc')

            prob = osqp.OSQP()
            prob.setup(sp.csc_matrix(H), f,
                       A_c, lb, ub,
                       warm_starting=True,
                       verbose=False,
                       max_iter=500,
                       eps_abs=1e-4, eps_rel=1e-4)
            res = prob.solve()

            if res.info.status in ('solved', 'solved_inaccurate'):
                u_flat = res.x
                # warm-start 시프트
                for k in range(N-1):
                    self.u_warm[k] = u_flat[(k+1)*self.NU:(k+2)*self.NU]
                self.u_warm[-1] = u_flat[(N-1)*self.NU:]

                av0    = float(u_flat[0])
                akap0  = float(u_flat[1])
                self.v     = np.clip(self.v     + self.Ts * av0,   self.v_min,    self.v_max)
                self.kappa = np.clip(self.kappa + self.Ts * akap0, self.kappa_min, self.kappa_max)
            else:
                av0 = 0.0

        except Exception as e:
            # OSQP 실패 시 단순 비례 제어 fallback
            dx = x_ref[0][0] - x0[0]
            dy = x_ref[0][1] - x0[1]
            dist = math.sqrt(dx*dx + dy*dy)
            target_psi = math.atan2(dy, dx)
            err_psi = math.atan2(math.sin(target_psi - x0[2]),
                                  math.cos(target_psi - x0[2]))
            self.v     = np.clip(-0.3, self.v_min, self.v_max)
            self.kappa = np.clip(err_psi * 0.5, self.kappa_min, self.kappa_max)

        v_cmd     = float(self.v)
        omega_cmd = v_cmd * float(self.kappa)
        return v_cmd, omega_cmd


# ============================================================
# 주차 매니저 노드
# ============================================================
class ParkingManagerNode(Node):

    def __init__(self) -> None:
        super().__init__('parking_manager_node')

        # ── 파라미터 ──
        self.declare_parameter('parking_trigger_dist',  20.0)   # [m]
        self.declare_parameter('parking_done_dist',      0.3)
        self.declare_parameter('parking_done_angle_deg', 5.0)
        self.declare_parameter('approach_speed',         3.0)   # [m/s]
        self.declare_parameter('parking_timeout_sec',  120.0)
        self.declare_parameter('publish_rate_hz',       20.0)
        self.declare_parameter('initial_mode',        'FORWARD')
        self.declare_parameter('wheelbase',             3.0)

        p = lambda n: self.get_parameter(n).value  # noqa
        self.park_trigger  = float(p('parking_trigger_dist'))
        self.park_done_d   = float(p('parking_done_dist'))
        self.park_done_ang = math.radians(float(p('parking_done_angle_deg')))
        self.approach_spd  = float(p('approach_speed'))
        self.park_timeout  = float(p('parking_timeout_sec'))
        self.wheelbase     = float(p('wheelbase'))

        # ── 상태 ──
        self.state = ParkState.FORWARD
        self.current_pose: Optional[Pose] = None
        self.current_v    = 0.0
        self.current_yaw  = 0.0
        self.parking_goal: Optional[Tuple[float,float,float]] = None  # (x, y, yaw)
        self.parking_path: List[Tuple] = []
        self.plan_start_time: Optional[float] = None
        self.park_start_time: Optional[float] = None
        self.forward_path:  Optional[Path] = None

        # 컨트롤러 초기화
        self.rti_nmpc = RTINMPCIoniq5()

        if _HAS_HASTAR:
            self.ha_cfg = HybridAStarConfig(
                xy_resolution=0.20,
                theta_resolution=math.radians(5),
                wheelbase=self.wheelbase,
                max_steer=math.radians(35),
                vehicle_length=4.635,
                vehicle_width=1.890,
                use_reed_shepp=True,
            )
            self.ha_planner = HybridAStarPlanner(self.ha_cfg)
        else:
            self.ha_planner = None
            self.get_logger().warn('hybrid_astar 없음 - 직선 경로 사용')

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── 구독 ──
        self.create_subscription(
            PoseStamped, '/Ego_pose', self._pose_cb, sensor_qos)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, sensor_qos)
        self.create_subscription(
            Path, '/local_path', self._path_cb, 10)
        self.create_subscription(
            PoseStamped, '/parking_goal', self._goal_cb, 10)
        self.create_subscription(
            String, '/drive_mode_cmd', self._mode_cmd_cb, 10)

        # ── 발행 ──
        self.accel_pub   = self.create_publisher(Accel,      '/Accel',         10)
        self.path_pub    = self.create_publisher(Path,        '/parking_path',  10)
        self.state_pub   = self.create_publisher(String,      '/parking_state', 10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/parking_markers', 10)

        # ── 타이머 ──
        hz = float(p('publish_rate_hz'))
        self.timer = self.create_wall_timer(1.0/hz, self._control_loop)

        # 초기 모드
        if p('initial_mode') == 'PARKING':
            self.state = ParkState.IDLE

        self.get_logger().info('주차 매니저 노드 시작 (Ioniq 5)')
        self.get_logger().info('  논문: Hybrid A* + RTI-NMPC + LTV-MPC')

    # =========================================================
    # 콜백
    # =========================================================

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg.pose
        q = msg.pose.orientation
        n = math.sqrt(q.x**2+q.y**2+q.z**2+q.w**2)
        if n > 1e-6:
            qz, qw = q.z/n, q.w/n
            self.current_yaw = math.atan2(2*qw*qz, 1-2*qz**2)

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_v = float(msg.twist.twist.linear.x)
        if self.current_pose is None:
            self.current_pose = msg.pose.pose

    def _path_cb(self, msg: Path) -> None:
        self.forward_path = msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        """외부에서 주차 목표 수신"""
        q = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y**2 + q.z**2))
        self.parking_goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            yaw)
        self.get_logger().info(
            f'주차 목표 수신: ({self.parking_goal[0]:.2f}, '
            f'{self.parking_goal[1]:.2f}, '
            f'{math.degrees(self.parking_goal[2]):.1f}°)')
        if self.state == ParkState.FORWARD:
            self.state = ParkState.APPROACH

    def _mode_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.upper()
        if cmd == 'FORWARD':
            self.state = ParkState.FORWARD
            self.rti_nmpc.reset()
        elif cmd == 'PARKING' and self.parking_goal:
            self.state = ParkState.PLAN
        elif cmd == 'IDLE':
            self.state = ParkState.IDLE

    # =========================================================
    # 메인 제어 루프
    # =========================================================

    def _control_loop(self) -> None:
        if self.current_pose is None:
            return

        self._publish_state()
        self._publish_visualization()

        if self.state == ParkState.FORWARD:
            self._forward_mode()
        elif self.state == ParkState.APPROACH:
            self._approach_mode()
        elif self.state == ParkState.PLAN:
            self._plan_parking()
        elif self.state == ParkState.REVERSE:
            self._reverse_parking()
        elif self.state == ParkState.FINE_TUNE:
            self._fine_tune()
        elif self.state == ParkState.DONE:
            self._send_stop()

    def _forward_mode(self) -> None:
        """전진 모드: LTV-MPC는 mpc_path_tracker_cpp가 처리, 여기선 목표 감시만"""
        if self.parking_goal is None:
            return
        px, py = self.current_pose.position.x, self.current_pose.position.y
        gx, gy = self.parking_goal[0], self.parking_goal[1]
        dist = math.hypot(gx - px, gy - py)
        if dist < self.park_trigger:
            self.get_logger().info(f'주차구역 접근: {dist:.1f}m → APPROACH 모드')
            self.state = ParkState.APPROACH

    def _approach_mode(self) -> None:
        """감속 접근"""
        if self.parking_goal is None:
            self.state = ParkState.FORWARD
            return
        px, py = self.current_pose.position.x, self.current_pose.position.y
        gx, gy = self.parking_goal[0], self.parking_goal[1]
        dist = math.hypot(gx - px, gy - py)

        if dist < 5.0:
            self.get_logger().info('경로 계획 시작 → PLAN 모드')
            self.state = ParkState.PLAN

    def _plan_parking(self) -> None:
        """
        Hybrid A* 경로 계획
        논문: 하이브리드 3편 (근본 논문 + 구조적 설계 + MPC 연결)
        """
        if self.parking_goal is None or self.current_pose is None:
            self.state = ParkState.FORWARD
            return

        sx = self.current_pose.position.x
        sy = self.current_pose.position.y
        sth = self.current_yaw
        gx, gy, gth = self.parking_goal

        self.get_logger().info(
            f'Hybrid A* 계획 중...\n'
            f'  시작: ({sx:.2f}, {sy:.2f}, {math.degrees(sth):.1f}°)\n'
            f'  목표: ({gx:.2f}, {gy:.2f}, {math.degrees(gth):.1f}°)')

        t_start = time.time()
        raw_path = None

        if self.ha_planner is not None:
            raw_path = self.ha_planner.plan(
                start_x=sx, start_y=sy, start_theta=sth,
                goal_x=gx,  goal_y=gy,  goal_theta=gth,
                map_origin_x=min(sx,gx)-10, map_origin_y=min(sy,gy)-10)

        t_elapsed = time.time() - t_start
        self.get_logger().info(f'Hybrid A* 완료: {t_elapsed*1000:.0f}ms')

        if raw_path is None or len(raw_path) < 2:
            self.get_logger().warn('Hybrid A* 실패 → 직선 경로 사용')
            raw_path = self._straight_path(sx, sy, sth, gx, gy, gth)

        # 경로 평활화 + 등간격 보간
        if self.ha_planner is not None:
            smooth = HybridAStarPlanner.smooth_path(raw_path)
            self.parking_path = HybridAStarPlanner.path_to_ros_waypoints(smooth, ds=0.3)
        else:
            self.parking_path = [(x, y, th) for x, y, th, *_ in raw_path]

        self._publish_parking_path()
        self.rti_nmpc.reset()
        self.park_start_time = time.time()
        self.state = ParkState.REVERSE
        self.get_logger().info(
            f'주차 경로: {len(self.parking_path)}개 웨이포인트 → REVERSE 모드')

    def _reverse_parking(self) -> None:
        """
        RTI-NMPC 후진 주차 추종
        논문: 2410.12170v1.pdf - Implicit Discretization RTI-NMPC
        """
        if not self.parking_path or self.current_pose is None:
            return

        # 타임아웃
        if (self.park_start_time and
                time.time() - self.park_start_time > self.park_timeout):
            self.get_logger().warn('주차 타임아웃 → FORWARD 복귀')
            self.state = ParkState.FORWARD
            return

        # 완료 확인
        if self._check_done():
            self.state = ParkState.FINE_TUNE
            return

        # 현재 상태 벡터
        x0 = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_yaw,
            self.current_v,
            self.rti_nmpc.kappa,
        ])

        # 참조 시퀀스 (가장 가까운 점 이후 N+1개)
        closest = self._find_closest(x0)
        x_ref = []
        for k in range(self.rti_nmpc.N + 1):
            idx = min(closest + k, len(self.parking_path) - 1)
            wx, wy, wth = self.parking_path[idx]
            x_ref.append(np.array([wx, wy, wth, -0.5, 0.0]))  # 후진 속도 -0.5

        # RTI-NMPC 제어 계산
        v_cmd, omega_cmd = self.rti_nmpc.compute(x0, x_ref)

        # Accel 발행
        accel_msg = Accel()
        target_v = v_cmd
        accel_msg.linear.x  = (target_v - self.current_v) * 3.0
        accel_msg.linear.x  = max(-3.0, min(1.0, accel_msg.linear.x))
        accel_msg.angular.z = omega_cmd
        self.accel_pub.publish(accel_msg)

    def _fine_tune(self) -> None:
        """최종 미세 조정 (저속, 위치 보정)"""
        if self.parking_goal is None:
            self.state = ParkState.DONE
            return

        gx, gy, gth = self.parking_goal
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        dist = math.hypot(gx-px, gy-py)
        d_ang = abs(math.atan2(math.sin(gth-self.current_yaw),
                                math.cos(gth-self.current_yaw)))

        if dist < self.park_done_d and d_ang < self.park_done_ang:
            self.get_logger().info(
                f'주차 완료! 거리={dist*100:.1f}cm 방향={math.degrees(d_ang):.1f}°')
            self._send_stop()
            self.state = ParkState.DONE
        else:
            # 저속 보정
            accel_msg = Accel()
            accel_msg.linear.x  = -0.1 if dist > 0.1 else 0.0
            accel_msg.angular.z = math.atan2(math.sin(gth-self.current_yaw),
                                              math.cos(gth-self.current_yaw)) * 0.3
            self.accel_pub.publish(accel_msg)

    def _send_stop(self) -> None:
        a = Accel()
        a.linear.x = -1.0 if abs(self.current_v) > 0.05 else 0.0
        self.accel_pub.publish(a)

    # =========================================================
    # 유틸리티
    # =========================================================

    def _check_done(self) -> bool:
        if self.parking_goal is None or self.current_pose is None:
            return False
        gx, gy, gth = self.parking_goal
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        dist = math.hypot(gx-px, gy-py)
        d_ang = abs(math.atan2(math.sin(gth-self.current_yaw),
                                math.cos(gth-self.current_yaw)))
        return dist < self.park_done_d * 2 and d_ang < self.park_done_ang * 2

    def _find_closest(self, x0: np.ndarray) -> int:
        if not self.parking_path:
            return 0
        best, best_d = 0, float('inf')
        for i, (wx, wy, _) in enumerate(self.parking_path):
            d = (wx-x0[0])**2 + (wy-x0[1])**2
            if d < best_d:
                best_d, best = d, i
        return best

    def _straight_path(self, sx, sy, sth, gx, gy, gth,
                       ds: float = 0.3) -> list:
        dist = math.hypot(gx-sx, gy-sy)
        n = max(int(dist/ds), 2)
        return [(sx + i/n*(gx-sx), sy + i/n*(gy-sy),
                 sth + i/n*(gth-sth), -1) for i in range(n+1)]

    def _publish_parking_path(self) -> None:
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for x, y, th in self.parking_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(th/2)
            ps.pose.orientation.w = math.cos(th/2)
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    def _publish_state(self) -> None:
        s = String()
        s.data = STATE_NAMES.get(self.state, 'UNKNOWN')
        self.state_pub.publish(s)

    def _publish_visualization(self) -> None:
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 주차 목표 마커
        if self.parking_goal:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns, m.id        = 'parking_goal', 0
            m.type            = Marker.ARROW
            m.action          = Marker.ADD
            gx, gy, gth = self.parking_goal
            m.pose.position.x = gx
            m.pose.position.y = gy
            m.pose.orientation.z = math.sin(gth/2)
            m.pose.orientation.w = math.cos(gth/2)
            m.scale.x, m.scale.y, m.scale.z = 2.0, 0.5, 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            markers.markers.append(m)

        # 주차 경로 마커
        if self.parking_path:
            m2 = Marker()
            m2.header.frame_id = 'map'
            m2.header.stamp    = now
            m2.ns, m2.id       = 'parking_path', 1
            m2.type            = Marker.LINE_STRIP
            m2.action          = Marker.ADD
            m2.scale.x         = 0.1
            m2.color.r, m2.color.b, m2.color.a = 0.0, 1.0, 1.0
            for x, y, _ in self.parking_path:
                p = Point(); p.x=x; p.y=y; p.z=0.1
                m2.points.append(p)
            markers.markers.append(m2)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
