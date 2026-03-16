#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
하이브리드 주차 시스템 Launch 파일
=====================================
전진: LTV-MPC (기존 mpc_path_tracker_cpp)
후진/주차: RTI-NMPC + Hybrid A*

노드 구성:
  1. mpc_path_tracker_cpp  - 전진 LTV-MPC 추종
  2. parking_spot_detector - 카메라 기반 주차구역 감지
  3. hybrid_mode_manager   - 모드 전환 + Hybrid A* 경로 계획
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('bisa')

    # ---- Launch 인수 ----
    cav_id_arg = DeclareLaunchArgument(
        'cav_id', default_value='1',
        description='CAV ID (1-4)')

    cam_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw',
        description='카메라 이미지 토픽')

    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode', default_value='FORWARD',
        description='초기 주행 모드 (FORWARD/REVERSE/IDLE)')

    debug_image_arg = DeclareLaunchArgument(
        'debug_image', default_value='true',
        description='디버그 이미지 발행 여부')

    cav_id  = LaunchConfiguration('cav_id')
    cam_top = LaunchConfiguration('camera_topic')
    init_m  = LaunchConfiguration('initial_mode')
    dbg_img = LaunchConfiguration('debug_image')

    # ================================================================
    # 1. LTV-MPC 경로 추종 노드 (전진용 - 기존 코드 재사용)
    # ================================================================
    ltv_mpc_node = Node(
        package='bisa',
        executable='mpc_path_tracker_cpp',
        name='ltv_mpc_tracker',
        output='screen',
        parameters=[{
            # 전진 모드 LTV-MPC 파라미터
            'use_ltv_mpc':        True,
            'prediction_horizon': 20,
            'time_step':          0.05,
            'wheelbase':          0.30,
            'weight_position':    20.0,
            'weight_heading':     8.0,
            'weight_curvature':   1.5,
            'weight_input':       1.0,
            'max_velocity':       1.5,
            'min_velocity':       0.2,
            'kappa_min_delta':   -0.9,
            'kappa_max_delta':    0.9,
            'kappa_blend_alpha':  0.15,
            'lateral_bound':      0.20,
            'w_lateral_slack_lin':  500.0,
            'w_lateral_slack_quad': 5000.0,
            # 출력 설정
            'publish_accel_cmd':  True,
        }],
        remappings=[
            ('/local_path',  '/local_path'),
            ('/Ego_pose',    '/Ego_pose'),
            ('/Accel',       '/ltv_accel_cmd'),   # 전진 MPC 전용 토픽
        ]
    )

    # ================================================================
    # 2. 주차구역 카메라 감지 노드
    # ================================================================
    parking_detector_node = Node(
        package='bisa',
        executable='parking_spot_detector_node.py',
        name='parking_spot_detector',
        output='screen',
        parameters=[{
            'image_topic':         cam_top,
            'publish_debug_image': dbg_img,
            # 흰색 감지 임계값
            'white_h_min': 0,
            'white_h_max': 180,
            'white_s_min': 0,
            'white_s_max': 50,
            'white_v_min': 170,
            'white_v_max': 255,
            # 주차구역 크기 [m] (실제 차량 스케일에 맞게 조정)
            'spot_width':  0.60,   # 소형 RC차 기준
            'spot_length': 0.90,
            'min_area_px': 500,
            'max_area_px': 100000,
            # 카메라 설치 파라미터
            'camera_height_m':   0.25,
            'camera_pitch_deg': -25.0,
            'camera_fov_deg':    70.0,
            # 신뢰도 임계값
            'confidence_threshold': 0.45,
        }]
    )

    # ================================================================
    # 3. 하이브리드 모드 매니저 노드
    # (주차 모드 전환 + Hybrid A* 경로 계획 + RTI-NMPC 조율)
    # ================================================================
    mode_manager_node = Node(
        package='bisa',
        executable='hybrid_mode_manager_node.py',
        name='hybrid_mode_manager',
        output='screen',
        parameters=[{
            'initial_mode':            init_m,
            # 모드 전환 파라미터
            'parking_trigger_distance': 5.0,   # [m] 주차구역 감지 후 전환 거리
            'parking_done_distance':    0.15,  # [m] 주차 완료 판단 거리
            'parking_done_angle_deg':   8.0,   # [deg] 주차 완료 판단 방향 오차
            'parking_speed':            0.25,  # [m/s] 주차 추종 속도
            'parking_timeout_sec':      90.0,  # [s] 주차 타임아웃
            'spot_confidence_threshold': 0.5,
            'publish_rate_hz':          20.0,
            # Hybrid A* 파라미터
            'ha_xy_resolution':   0.08,
            'ha_theta_deg':       5.0,
            'ha_wheelbase':       0.30,
            'ha_max_steer_deg':  35.0,
            'ha_vehicle_length':  0.45,
            'ha_vehicle_width':   0.25,
            'ha_use_reed_shepp':  True,
        }]
    )

    # ================================================================
    # Launch Description
    # ================================================================
    return LaunchDescription([
        cav_id_arg,
        cam_topic_arg,
        initial_mode_arg,
        debug_image_arg,
        ltv_mpc_node,
        parking_detector_node,
        mode_manager_node,
    ])
