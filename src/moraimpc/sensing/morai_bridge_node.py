#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI 시뮬레이터 토픽 브릿지 노드
====================================
MORAI 토픽 ↔ 기존 henes 시스템 토픽 변환

MORAI → 시스템:
  /Ego_topic (EgoVehicleStatus) → /Ego_pose, /odometry/filtered, /morai/heading
  /imu       (sensor_msgs/Imu)  → /handsfree/imu (패스스루)
  /gps       (NavSatFix)        → /gnss_rover/fix (패스스루)
  /image_raw (Image)            → /camera/image_raw (패스스루)

시스템 → MORAI:
  /cmd_vel (Twist) OR /Accel (Accel) → /ctrl_cmd (MoraiCtrlCmd)

Ioniq 5 제원 반영:
  wheelbase = 3.0m, max_steer = 35°, max_v = 30 m/s
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                        QoSProfile, ReliabilityPolicy)

from geometry_msgs.msg import (PoseStamped, Pose, Twist,
                                TwistWithCovarianceStamped, Accel)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, Image
from std_msgs.msg import Float64, Header

# MORAI 메시지 (morai_msgs 패키지 필요)
try:
    from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
    _HAS_MORAI_MSGS = True
except ImportError:
    _HAS_MORAI_MSGS = False
    import warnings
    warnings.warn("morai_msgs 없음 - EgoVehicleStatus stub 사용")


# ============================================================
# Ioniq 5 파라미터
# ============================================================
class Ioniq5Params:
    WHEELBASE    = 3.000   # [m]
    MAX_STEER    = math.radians(35.0)
    MAX_VELOCITY = 30.0    # [m/s]
    MIN_VELOCITY = -3.0    # [m/s] 후진
    MAX_ACCEL    =  3.0    # [m/s²]
    MAX_KAPPA    = math.tan(MAX_STEER) / WHEELBASE  # ≈ 0.234


# ============================================================
# MORAI 브릿지 노드
# ============================================================
class MoraiBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('morai_bridge_node')

        # ── 파라미터 ──
        self.declare_parameter('ego_topic',       '/Ego_topic')
        self.declare_parameter('ctrl_topic',      '/ctrl_cmd')
        self.declare_parameter('imu_in_topic',    '/imu')
        self.declare_parameter('imu_out_topic',   '/handsfree/imu')
        self.declare_parameter('gps_in_topic',    '/gps')
        self.declare_parameter('gps_out_topic',   '/gnss_rover/fix')
        self.declare_parameter('cam_in_topic',    '/image_raw')
        self.declare_parameter('cam_out_topic',   '/camera/image_raw')
        self.declare_parameter('use_accel_cmd',   True)
        self.declare_parameter('wheelbase',       Ioniq5Params.WHEELBASE)
        self.declare_parameter('max_steer_deg',   35.0)

        p = lambda n: self.get_parameter(n).value  # noqa

        self.wheelbase = float(p('wheelbase'))
        self.max_steer = math.radians(float(p('max_steer_deg')))
        self.use_accel_cmd = bool(p('use_accel_cmd'))

        # 현재 속도 (cmd_vel → ctrl_cmd 변환용)
        self._current_v = 0.0
        self._current_heading_rad = 0.0

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable_qos = QoSProfile(depth=10)

        # ── 구독: MORAI → 시스템 ──
        if _HAS_MORAI_MSGS:
            self.create_subscription(
                EgoVehicleStatus, p('ego_topic'),
                self._ego_cb, reliable_qos)
        else:
            self.get_logger().warn('morai_msgs 없음: /Ego_topic 구독 불가')

        self.create_subscription(Imu,       p('imu_in_topic'), self._imu_cb,    sensor_qos)
        self.create_subscription(NavSatFix, p('gps_in_topic'), self._gps_cb,    sensor_qos)
        self.create_subscription(Image,     p('cam_in_topic'), self._camera_cb, sensor_qos)

        # ── 구독: 시스템 → MORAI ──
        if self.use_accel_cmd:
            self.create_subscription(Accel,  '/Accel',    self._accel_cmd_cb, reliable_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, reliable_qos)

        # ── 발행: 시스템 토픽 ──
        self.ego_pose_pub   = self.create_publisher(PoseStamped, '/Ego_pose',   10)
        self.odom_pub       = self.create_publisher(Odometry,    '/odometry/filtered', 10)
        self.heading_pub    = self.create_publisher(Float64,     '/morai/heading_deg', 10)
        self.imu_pub        = self.create_publisher(Imu,         p('imu_out_topic'), 10)
        self.gps_pub        = self.create_publisher(NavSatFix,   p('gps_out_topic'),  10)
        self.vel_pub        = self.create_publisher(
            TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', 10)
        self.cam_pub        = self.create_publisher(Image,       p('cam_out_topic'),  10)

        # ── 발행: MORAI 제어 ──
        if _HAS_MORAI_MSGS:
            self.ctrl_pub = self.create_publisher(CtrlCmd, p('ctrl_topic'), 10)

        self.get_logger().info('MORAI 브릿지 노드 시작 (Ioniq 5 파라미터)')
        self.get_logger().info(f'  Wheelbase: {self.wheelbase}m')
        self.get_logger().info(f'  Max steer: {math.degrees(self.max_steer):.1f}°')
        self.get_logger().info(f'  morai_msgs: {"사용 가능" if _HAS_MORAI_MSGS else "없음"}')

    # =========================================================
    # MORAI → 시스템 변환
    # =========================================================

    def _ego_cb(self, msg) -> None:
        """
        EgoVehicleStatus → PoseStamped + Odometry + heading

        MORAI EgoVehicleStatus 주요 필드:
          position_x/y/z : UTM 좌표 [m]
          velocity_x/y/z : 속도 [m/s] (차량 로컬 프레임 또는 전역)
          heading        : 방향각 [deg] (0=North 또는 0=East, 버전별 다름)
          accel_x/y/z    : 가속도 [m/s²]
          angular_velocity_z : 요 레이트 [rad/s]
        """
        now = self.get_clock().now().to_msg()

        # 방향각 (MORAI는 0=North CW 방식 → ROS는 0=East CCW)
        # MORAI heading: 0=North, CW 방향
        # ROS yaw: 0=East, CCW 방향
        # 변환: ros_yaw = π/2 - morai_heading_rad
        heading_deg = float(getattr(msg, 'heading', 0.0))
        heading_rad = math.radians(heading_deg)
        ros_yaw = math.pi / 2.0 - heading_rad
        ros_yaw = math.atan2(math.sin(ros_yaw), math.cos(ros_yaw))
        self._current_heading_rad = ros_yaw

        # 속도 계산
        vx = float(getattr(msg, 'velocity_x', 0.0))
        vy = float(getattr(msg, 'velocity_y', 0.0))
        speed = math.hypot(vx, vy)

        # 전진/후진 판단 (velocity_x 부호 사용)
        v_signed = vx if abs(vx) > 0.01 else speed
        self._current_v = v_signed

        px = float(getattr(msg, 'position_x', 0.0))
        py = float(getattr(msg, 'position_y', 0.0))
        pz = float(getattr(msg, 'position_z', 0.0))

        # 쿼터니언 (yaw만)
        half = ros_yaw / 2.0
        qz = math.sin(half)
        qw = math.cos(half)

        # ── PoseStamped 발행 (/Ego_pose) ──
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = px
        pose_msg.pose.position.y = py
        pose_msg.pose.position.z = pz
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.ego_pose_pub.publish(pose_msg)

        # ── Odometry 발행 (/odometry/filtered) ──
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'map'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose = pose_msg.pose
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = float(getattr(msg, 'angular_velocity_z', 0.0))
        self.odom_pub.publish(odom)

        # ── 헤딩 발행 ──
        h = Float64()
        h.data = math.degrees(ros_yaw)
        self.heading_pub.publish(h)

        # ── GPS 속도 발행 (ESKF 입력용) ──
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp    = now
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.twist.linear.x = vx
        vel_msg.twist.twist.linear.y = vy
        sigma_v = 0.1
        vel_msg.twist.covariance[0]  = sigma_v**2
        vel_msg.twist.covariance[7]  = sigma_v**2
        vel_msg.twist.covariance[35] = math.radians(1.0)**2
        self.vel_pub.publish(vel_msg)

    def _imu_cb(self, msg: Imu) -> None:
        """IMU 패스스루 (/imu → /handsfree/imu)"""
        self.imu_pub.publish(msg)

    def _gps_cb(self, msg: NavSatFix) -> None:
        """GPS 패스스루 (/gps → /gnss_rover/fix)"""
        self.gps_pub.publish(msg)

    def _camera_cb(self, msg: Image) -> None:
        """카메라 패스스루 (/image_raw → /camera/image_raw)"""
        self.cam_pub.publish(msg)

    # =========================================================
    # 시스템 → MORAI 변환
    # =========================================================

    def _accel_cmd_cb(self, msg: Accel) -> None:
        """
        /Accel (geometry_msgs/Accel) → /ctrl_cmd (MoraiCtrlCmd)
        Accel.linear.x  = 종방향 가속도 [m/s²]
        Accel.angular.z = 각속도 명령 [rad/s]
        """
        if not _HAS_MORAI_MSGS:
            return

        longi_accel = float(msg.linear.x)
        omega_cmd   = float(msg.angular.z)

        # omega = v * kappa → kappa = omega / v (v≠0)
        steering_rad = self._omega_to_steer(omega_cmd, self._current_v)

        self._publish_ctrl(longi_accel, steering_rad)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """
        /cmd_vel (Twist) → /ctrl_cmd (MoraiCtrlCmd)
        선형 속도 명령 → 가속도로 변환 (비례)
        """
        if not _HAS_MORAI_MSGS or self.use_accel_cmd:
            return

        target_v    = float(msg.linear.x)
        omega_cmd   = float(msg.angular.z)
        longi_accel = (target_v - self._current_v) * 2.0   # 간단한 비례 제어
        longi_accel = max(-5.0, min(3.0, longi_accel))

        steering_rad = self._omega_to_steer(omega_cmd, self._current_v)
        self._publish_ctrl(longi_accel, steering_rad)

    def _omega_to_steer(self, omega: float, v: float) -> float:
        """각속도 명령 → 조향각 변환"""
        if abs(v) < 0.1:
            kappa = omega / 0.1
        else:
            kappa = omega / v
        kappa = max(-Ioniq5Params.MAX_KAPPA,
                    min(Ioniq5Params.MAX_KAPPA, kappa))
        steer = math.atan(kappa * self.wheelbase)
        return max(-self.max_steer, min(self.max_steer, steer))

    def _publish_ctrl(self, longi_accel: float, steering_rad: float) -> None:
        """MoraiCtrlCmd 발행"""
        ctrl = CtrlCmd()
        ctrl.longi_accel = float(longi_accel)
        ctrl.steering    = float(steering_rad)
        ctrl.brake       = max(0.0, -longi_accel * 0.3) if longi_accel < 0 else 0.0
        self.ctrl_pub.publish(ctrl)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoraiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
