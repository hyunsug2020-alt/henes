#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dual_gps_imu_stack.launch.py
==============================
GPS1 스택 런치 (ESKF 없이 GPS 직접 사용)

노드 구성:
  - gnss_nmea_node (gnss_left / gps1 / /dev/henes_gps) → /gnss_left/fix + /gnss_left/fix_velocity
  - gps_odom_node  (GPS → /odometry/filtered + /utm_origin)
  - ntrip_ros (NTRIP RTK 보정)
  - rosbridge WebSocket (Foxglove)
"""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LAUNCH_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(LAUNCH_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(LAUNCH_REPO_ROOT))

from sensing.device_config import (
    default_device_config_path,
    format_device_resolution,
    load_device_config,
    resolve_device_selection,
)


def _resolve_devices(context):
    config_file = (LaunchConfiguration('device_config_file').perform(context).strip()
                   or default_device_config_path())
    config = load_device_config(config_file)
    actions = []

    # GPS 좌 (gps1 → gnss_left)
    sel_left = resolve_device_selection(
        config, 'gps',
        requested_name=LaunchConfiguration('left_gps_name').perform(context).strip(),
        requested_path=LaunchConfiguration('left_gps_device').perform(context).strip(),
    )
    actions.append(SetLaunchConfiguration('resolved_left_gps', sel_left['path']))
    print(f'[gps1_eskf] {format_device_resolution(sel_left, "GPS1 (gnss_left)")}')

    # IMU
    sel_imu = resolve_device_selection(
        config, 'imu',
        requested_name=LaunchConfiguration('imu_name').perform(context).strip(),
        requested_path=LaunchConfiguration('imu_device').perform(context).strip(),
    )
    actions.append(SetLaunchConfiguration('resolved_imu', sel_imu['path']))
    print(f'[gps1_eskf] {format_device_resolution(sel_imu, "IMU")}')

    return actions


def generate_launch_description():

    # ── 런치 인수 ──────────────────────────────────────────────
    device_config_file  = DeclareLaunchArgument('device_config_file',  default_value=default_device_config_path())
    left_gps_name       = DeclareLaunchArgument('left_gps_name',       default_value='gps1')
    left_gps_device     = DeclareLaunchArgument('left_gps_device',     default_value='')
    imu_name            = DeclareLaunchArgument('imu_name',            default_value='imu1')
    imu_device          = DeclareLaunchArgument('imu_device',          default_value='')
    imu_baudrate        = DeclareLaunchArgument('imu_baudrate',        default_value='921600')
    enable_imu          = DeclareLaunchArgument('enable_imu',          default_value='false')
    enable_ntrip        = DeclareLaunchArgument('enable_ntrip',        default_value='true')
    enable_foxglove     = DeclareLaunchArgument('enable_foxglove',     default_value='true')
    utm_zone            = DeclareLaunchArgument('utm_zone',            default_value='52')

    device_resolver = OpaqueFunction(function=_resolve_devices)

    # ================================================================
    # GPS1 (gnss_left) - NMEA 시리얼 파서
    # → /gnss_left/fix  + /gnss_left/fix_velocity
    # ================================================================
    left_gps_node = Node(
        package='jeju',
        executable='gnss_nmea_node.py',
        name='gnss_nmea_left',
        output='screen',
        parameters=[{
            'port':       LaunchConfiguration('resolved_left_gps'),
            'baud':       460800,
            'fix_topic':  '/gnss_left/fix',
            'vel_topic':  '/gnss_left/fix_velocity',
            'frame_id':   'gnss_left',
            'publish_hz': 10.0,
            'rtcm_topic': '/ntrip_client/rtcm',
        }],
    )

    # ================================================================
    # IMU 드라이버
    # ================================================================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_imu_driver'), 'launch', 'imu.launch.py'
            ])
        ),
        launch_arguments={
            'imu_device':   LaunchConfiguration('resolved_imu'),
            'imu_baudrate': LaunchConfiguration('imu_baudrate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    # ================================================================
    # GPS Odom 노드 (GPS 직접 → /odometry/filtered + /utm_origin)
    # ESKF 없이 GPS 위치/속도를 바로 사용
    # ================================================================
    gps_odom_node = Node(
        package='jeju',
        executable='gps_odom_node',
        name='gps_odom_node',
        output='screen',
        parameters=[{
            'utm_zone':        LaunchConfiguration('utm_zone'),
            'gps_fix_topic':   '/gnss_left/fix',
            'gps_vel_topic':   '/gnss_left/fix_velocity',
            'heading_min_m':   0.05,  # RTK 위치 기반 헤딩 최소 이동 거리 [m]
        }],
    )

    # ================================================================
    # NTRIP 클라이언트 (RTK 보정)
    # ================================================================
    ntrip_node = Node(
        package='ntrip_ros',
        executable='ntripclient',
        name='ntrip_ros',
        output='screen',
        parameters=[{
            'ntrip_server':     'RTS1.ngii.go.kr:2101',
            'ntrip_user':       'kjb121000',
            'ntrip_pass':       'ngii',
            'ntrip_stream':     'VRS-RTCM31',
            'rtcm_topic':       '/ntrip_client/rtcm',
            'fix_topic':        '/gnss_left/fix',
            'require_live_gga': True,
            'nmea_gga':         '',
        }],
        condition=IfCondition(LaunchConfiguration('enable_ntrip')),
    )

    # ================================================================
    # Foxglove rosbridge WebSocket
    # ================================================================
    rosbridge_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={'port': '9090'}.items(),
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
    )

    # ================================================================
    # 로컬 경로 발행 노드 (글로벌 경로 → 슬라이딩 윈도우 로컬 경로)
    # /global_path + /odometry/filtered → /local_path
    # ================================================================
    local_path_node = Node(
        package='jeju',
        executable='local_path_pub_node',
        name='local_path_pub',
        output='screen',
        parameters=[{
            'local_path_size':       120,
            'search_window':         150,
            'max_index_step':         20,
            'reinit_dist_thresh':      2.0,
            'smooth_window':           2,
            'corner_smooth_window':    0,
            'corner_kappa_thresh':     0.08,
        }],
    )

    # ================================================================
    # 센서 모니터 GUI
    # ================================================================
    sensor_monitor_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='sensor_monitor',
        output='screen',
        parameters=[{
            'imu_topic':      '/handsfree/imu',
            'gps1_label':     'GPS1',
            'gps1_fix_topic': '/gnss_left/fix',
            'heading_topic':  '/gps_odom/heading_deg',
            'window_name':    'HENES Sensor Monitor',
            'refresh_hz':     15.0,
        }],
    )

    return LaunchDescription([
        # 인수
        device_config_file,
        left_gps_name,
        left_gps_device,
        imu_name,
        imu_device,
        imu_baudrate,
        enable_imu,
        enable_ntrip,
        enable_foxglove,
        utm_zone,
        # 디바이스 탐색
        device_resolver,
        # 노드
        left_gps_node,
        imu_launch,
        gps_odom_node,
        ntrip_node,
        rosbridge_node,
        local_path_node,
        sensor_monitor_node,
    ])
