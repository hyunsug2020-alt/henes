#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq 5 전체 스택 런치
==============================
노드 구성:
  1. morai_bridge_node      - MORAI 토픽 변환
  2. eskf_node              - GPS+IMU ESKF 헤딩
  3. mpc_path_tracker_cpp   - LTV-MPC 전진 추종
  4. parking_manager_node   - Hybrid A* + RTI-NMPC 주차
  5. iridescence_gui_node   - 통합 시각화 GUI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('moraimpc')
    config = os.path.join(pkg, 'config', 'ioniq5.yaml')

    # ── Launch 인수 ──
    use_sim_time   = DeclareLaunchArgument('use_sim_time',  default_value='false')
    enable_gui     = DeclareLaunchArgument('enable_gui',    default_value='true')
    enable_parking = DeclareLaunchArgument('enable_parking',default_value='true')
    initial_mode   = DeclareLaunchArgument('initial_mode',  default_value='FORWARD')
    ego_topic      = DeclareLaunchArgument('ego_topic',     default_value='/Ego_topic')
    ctrl_topic     = DeclareLaunchArgument('ctrl_topic',    default_value='/ctrl_cmd')

    # ================================================================
    # 1. MORAI 브릿지 (토픽 변환)
    # ================================================================
    morai_bridge = Node(
        package='moraimpc',
        executable='morai_bridge_node',
        name='morai_bridge_node',
        output='screen',
        parameters=[config, {
            'ego_topic':    LaunchConfiguration('ego_topic'),
            'ctrl_topic':   LaunchConfiguration('ctrl_topic'),
            'imu_in_topic': '/imu',
            'gps_in_topic': '/gps',
            'use_accel_cmd': True,
            'wheelbase':    3.0,
            'max_steer_deg': 35.0,
        }],
    )

    # ================================================================
    # 2. ESKF (GPS+IMU 헤딩 융합)
    # 논문: 2406.06427v3.pdf
    # ================================================================
    eskf = Node(
        package='moraimpc',
        executable='eskf_node',
        name='eskf_node',
        output='screen',
        parameters=[config, {
            'imu_topic':              '/handsfree/imu',
            'gps_fix_topic':          '/gnss_rover/fix',
            'gps_vel_topic':          '/ublox_gps/fix_velocity',
            'dual_heading_topic':     '/morai/heading_deg',
            'dual_heading_valid_topic':'/morai/heading_valid',
            'dual_heading_accuracy_topic':'/morai/heading_accuracy_deg',
            'sigma_gps_pos':  1.0,    # MORAI GPS 정확도
            'sigma_gps_vel':  0.2,
            'sigma_heading':  0.05,
            'utm_zone':       52,
        }],
    )

    # ================================================================
    # 3. LTV-MPC 전진 경로 추종
    # 논문: Linear_Time-Varying_MPC.pdf (Gutjahr 2017)
    # 기존 bisa 패키지 mpc_path_tracker_cpp 재사용
    # ================================================================
    ltv_mpc = Node(
        package='bisa',
        executable='mpc_path_tracker_cpp',
        name='ltv_mpc_tracker',
        output='screen',
        parameters=[{
            'use_ltv_mpc':        True,
            'prediction_horizon': 20,
            'time_step':          0.2,
            'wheelbase':          3.0,        # Ioniq 5
            'weight_position':    20.0,
            'weight_heading':     8.0,
            'weight_curvature':   1.5,
            'weight_input':       1.0,
            'max_velocity':       30.0,
            'min_velocity':       0.5,
            'kappa_min_delta':   -0.234,       # tan(35°)/3.0
            'kappa_max_delta':    0.234,
            'kappa_blend_alpha':  0.15,
            'lateral_bound':      0.30,
            'w_lateral_slack_lin':  500.0,
            'w_lateral_slack_quad': 5000.0,
            'publish_accel_cmd':  True,
        }],
        remappings=[
            ('/Ego_pose',   '/Ego_pose'),
            ('/local_path', '/local_path'),
            ('/Accel',      '/Accel'),
        ],
    )

    # ================================================================
    # 4. 주차 매니저 (Hybrid A* + RTI-NMPC)
    # 논문: 하이브리드 3편 + 2410.12170v1.pdf
    # ================================================================
    parking = Node(
        package='moraimpc',
        executable='parking_manager_node',
        name='parking_manager_node',
        output='screen',
        parameters=[config, {
            'initial_mode':         LaunchConfiguration('initial_mode'),
            'parking_trigger_dist': 20.0,
            'parking_done_dist':     0.3,
            'parking_done_angle_deg':5.0,
            'approach_speed':        3.0,
            'parking_timeout_sec': 180.0,
            'wheelbase':             3.0,
        }],
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
            LaunchConfiguration('enable_parking')),
    )

    # ================================================================
    # 5. Iridescence 통합 GUI
    # ================================================================
    gui = Node(
        package='moraimpc',
        executable='iridescence_gui_node',
        name='iridescence_gui_node',
        output='screen',
        parameters=[{}],
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(
            LaunchConfiguration('enable_gui')),
    )

    return LaunchDescription([
        use_sim_time,
        enable_gui,
        enable_parking,
        initial_mode,
        ego_topic,
        ctrl_topic,
        morai_bridge,
        eskf,
        ltv_mpc,
        parking,
        gui,
    ])
