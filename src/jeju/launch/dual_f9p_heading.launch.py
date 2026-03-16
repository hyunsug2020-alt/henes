#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU + Dual GPS (F9P) 헤딩 런치 파일
=====================================
실행 노드:
  - ublox_dgnss_node (base GPS)
  - ublox_dgnss_node (rover GPS)
  - dual_gnss_heading_node   : 듀얼 GPS 헤딩 계산
  - imu_driver               : IMU 드라이버
  - gnss_velocity_bridge_node: GPS 속도 브릿지 (헤딩 방향 분석)
  - imu_axis_check_gui       : IMU 축 확인 GUI
  - status_info_node         : GPS/IMU → odometry 퍼블리셔
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

LAUNCH_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(LAUNCH_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(LAUNCH_REPO_ROOT))

from control.device_config import (
    default_device_config_path,
    format_device_resolution,
    load_device_config,
    resolve_device_selection,
)


def _resolve_dual_gps_devices(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    actions = []

    for label, name_key, path_key, resolved_key in (
        ('base',  'base_gps_name',  'base_device',  'resolved_base_device'),
        ('rover', 'rover_gps_name', 'rover_device', 'resolved_rover_device'),
    ):
        selection = resolve_device_selection(
            config,
            'gps',
            requested_name=LaunchConfiguration(name_key).perform(context).strip(),
            requested_path=LaunchConfiguration(path_key).perform(context).strip(),
        )
        actions.append(SetLaunchConfiguration(resolved_key, selection['path']))
        print(f'[jeju] {format_device_resolution(selection, f"{label} gps")}')

    imu_selection = resolve_device_selection(
        config,
        'imu',
        requested_name=LaunchConfiguration('imu_name').perform(context).strip(),
        requested_path=LaunchConfiguration('imu_device').perform(context).strip(),
    )
    actions.append(SetLaunchConfiguration('resolved_imu_device', imu_selection['path']))
    print(f'[jeju] {format_device_resolution(imu_selection, "imu device")}')

    print('[jeju] INFO: dual heading needs GPS2(base) -> GPS1(rover) RTCM on UART2, '
          'or an equivalent RTCM feed into the rover.')
    return actions


def generate_launch_description():

    # ---- Launch 인수 선언 ----
    device_config_file   = DeclareLaunchArgument('device_config_file',   default_value=default_device_config_path())
    device_family        = DeclareLaunchArgument('device_family',         default_value='F9P')
    base_namespace       = DeclareLaunchArgument('base_namespace',        default_value='gnss_base')
    rover_namespace      = DeclareLaunchArgument('rover_namespace',       default_value='gnss_rover')
    base_gps_name        = DeclareLaunchArgument('base_gps_name',         default_value='gps2')
    rover_gps_name       = DeclareLaunchArgument('rover_gps_name',        default_value='gps1')
    base_device          = DeclareLaunchArgument('base_device',           default_value='')
    rover_device         = DeclareLaunchArgument('rover_device',          default_value='')
    base_device_serial   = DeclareLaunchArgument('base_device_serial',    default_value='')
    rover_device_serial  = DeclareLaunchArgument('rover_device_serial',   default_value='')
    base_frame_id        = DeclareLaunchArgument('base_frame_id',         default_value='gnss_base')
    rover_frame_id       = DeclareLaunchArgument('rover_frame_id',        default_value='gnss_rover')
    rover_rtcm_topic     = DeclareLaunchArgument('rover_rtcm_topic',      default_value='/gnss_base/rtcm')
    imu_name             = DeclareLaunchArgument('imu_name',              default_value='imu1')
    imu_device           = DeclareLaunchArgument('imu_device',            default_value='')
    imu_baudrate         = DeclareLaunchArgument('imu_baudrate',          default_value='921600')
    enable_imu           = DeclareLaunchArgument('enable_imu',            default_value='true')
    enable_status_info   = DeclareLaunchArgument('enable_status_info',    default_value='true')
    enable_gps_rtk_gui   = DeclareLaunchArgument('enable_gps_rtk_gui',   default_value='false')  # 통합 GUI로 대체
    enable_imu_gui       = DeclareLaunchArgument('enable_imu_gui',        default_value='false')  # 통합 GUI로 대체
    enable_sensor_monitor = DeclareLaunchArgument('enable_sensor_monitor', default_value='true')
    enable_eskf          = DeclareLaunchArgument('enable_eskf',           default_value='true')
    imu_topic            = DeclareLaunchArgument('imu_topic',             default_value='/handsfree/imu')
    tilt_threshold_deg   = DeclareLaunchArgument('tilt_threshold_deg',    default_value='4.0')
    turn_threshold_rad_s = DeclareLaunchArgument('turn_threshold_rad_s',  default_value='0.15')

    fix_velocity_input_topic  = DeclareLaunchArgument(
        'fix_velocity_input_topic',  default_value='/gnss_rover/ubx_nav_vel_ned')
    fix_velocity_output_topic = DeclareLaunchArgument(
        'fix_velocity_output_topic', default_value='/ublox_gps/fix_velocity')

    status_params_file = DeclareLaunchArgument(
        'status_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'), 'config', 'params.yaml']))
    heading_params_file = DeclareLaunchArgument(
        'heading_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'), 'config', 'dual_f9p_heading.yaml']))

    device_resolver = OpaqueFunction(function=_resolve_dual_gps_devices)

    # ================================================================
    # Base GPS (GPS2) - ublox_dgnss_node ComposableNode
    # ================================================================
    base_gps_container = ComposableNodeContainer(
        name='ublox_base_container',
        namespace=LaunchConfiguration('base_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDgnssNode',
                name='ublox_dgnss_node',
                namespace=LaunchConfiguration('base_namespace'),
                parameters=[{
                    'device_family':        LaunchConfiguration('device_family'),
                    'device':               LaunchConfiguration('resolved_base_device'),
                    'device_serial_string': LaunchConfiguration('base_device_serial'),
                    'frame_id':             LaunchConfiguration('base_frame_id'),
                }],
            ),
        ],
        output='screen',
    )

    # ================================================================
    # Rover GPS (GPS1) - ublox_dgnss_node ComposableNode
    # ================================================================
    rover_gps_container = ComposableNodeContainer(
        name='ublox_rover_container',
        namespace=LaunchConfiguration('rover_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDgnssNode',
                name='ublox_dgnss_node',
                namespace=LaunchConfiguration('rover_namespace'),
                parameters=[{
                    'device_family':        LaunchConfiguration('device_family'),
                    'device':               LaunchConfiguration('resolved_rover_device'),
                    'device_serial_string': LaunchConfiguration('rover_device_serial'),
                    'frame_id':             LaunchConfiguration('rover_frame_id'),
                    'rtcm_input_topic':     LaunchConfiguration('rover_rtcm_topic'),
                }],
            ),
        ],
        output='screen',
    )

    # ================================================================
    # 듀얼 GPS 헤딩 계산 노드
    # ================================================================
    dual_heading_node = Node(
        package='jeju',
        executable='dual_gnss_heading_node.py',
        name='dual_gnss_heading_node',
        output='screen',
        parameters=[LaunchConfiguration('heading_params_file')],
    )

    # ================================================================
    # IMU 드라이버
    # ================================================================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_imu_driver'), 'launch', 'imu.launch.py'])
        ),
        launch_arguments={
            'imu_device':   LaunchConfiguration('resolved_imu_device'),
            'imu_baudrate': LaunchConfiguration('imu_baudrate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    # ================================================================
    # GPS 속도 브릿지 (헤딩 방향 분석용)
    # paper/gnss_velocity_bridge_node.py 기반
    # UBX NAV-VELNED → /ublox_gps/fix_velocity 변환
    # ================================================================
    fix_velocity_bridge = Node(
        package='jeju',
        executable='gnss_velocity_bridge_node.py',
        name='gnss_velocity_bridge_node',
        output='screen',
        parameters=[{
            'input_topic':  LaunchConfiguration('fix_velocity_input_topic'),
            'output_topic': LaunchConfiguration('fix_velocity_output_topic'),
            'frame_id':     'odom',
        }],
    )

    # ================================================================
    # IMU 축 확인 GUI
    # paper/imu_axis_check_gui.py 기반 (tkinter GUI)
    # ================================================================
    imu_gui_node = Node(
        package='jeju',
        executable='imu_axis_check_gui.py',
        name='imu_axis_check_gui',
        output='screen',
        parameters=[{
            'imu_topic':             LaunchConfiguration('imu_topic'),
            'tilt_threshold_deg':    ParameterValue(LaunchConfiguration('tilt_threshold_deg'),    value_type=float),
            'turn_threshold_rad_s':  ParameterValue(LaunchConfiguration('turn_threshold_rad_s'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('enable_imu_gui')),
    )

    # ================================================================
    # ESKF 노드 (GPS+IMU 융합, 정밀 헤딩)
    # 논문: 2406.06427v3.pdf 기반
    # /odometry/filtered 발행 (status_info_node 대체)
    # ================================================================
    eskf_node = Node(
        package='jeju',
        executable='eskf_node.py',
        name='eskf_node',
        output='screen',
        parameters=[{
            'imu_topic':                  '/handsfree/imu',
            'gps_fix_topic':              '/gnss_rover/fix',
            'gps_vel_topic':              LaunchConfiguration('fix_velocity_output_topic'),
            'dual_heading_topic':         '/dual_f9p/heading',
            'dual_heading_valid_topic':   '/dual_f9p/heading_valid',
            'dual_heading_accuracy_topic':'/dual_f9p/heading_accuracy_deg',
            # 노이즈 튜닝
            'sigma_accel':        0.05,
            'sigma_gyro':         0.005,
            'sigma_accel_bias':   0.001,
            'sigma_gyro_bias':    0.0001,
            'sigma_gps_pos':      0.5,
            'sigma_gps_vel':      0.1,
            'sigma_heading':      0.02,
            'yaw_offset_deg':     0.0,
            'utm_zone':           52,
        }],
        condition=IfCondition(LaunchConfiguration('enable_eskf')),
    )

    # ================================================================
    # GPS/IMU 상태 정보 노드 (ESKF 미사용 시 fallback)
    # ================================================================
    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
        parameters=[
            LaunchConfiguration('status_params_file'),
            {
                'gps_fix_topic':            '/gnss_rover/fix',
                'gps_vel_topic':            LaunchConfiguration('fix_velocity_output_topic'),
                'imu_topic':                '/handsfree/imu',
                'dual_heading_topic':       '/dual_f9p/heading',
                'dual_heading_valid_topic': '/dual_f9p/heading_valid',
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_status_info')),
    )

    # ================================================================
    # 통합 센서 모니터 GUI (IMU + GPS + ESKF 헤딩 한 창)
    # ================================================================
    sensor_monitor_node = Node(
        package='jeju',
        executable='sensor_monitor_gui.py',
        name='sensor_monitor_gui',
        output='screen',
        parameters=[{
            'imu_topic':                   '/handsfree/imu',
            'gps_fix_topic':               '/gnss_rover/fix',
            'gps_vel_topic':               LaunchConfiguration('fix_velocity_output_topic'),
            'dual_heading_topic':          '/dual_f9p/heading',
            'dual_heading_valid_topic':    '/dual_f9p/heading_valid',
            'dual_heading_acc_topic':      '/dual_f9p/heading_accuracy_deg',
            'dual_baseline_topic':         '/dual_f9p/baseline_length_m',
            'eskf_heading_topic':          '/eskf/heading_deg',
            'eskf_cov_topic':              '/eskf/covariance_trace',
            'odom_topic':                  '/odometry/filtered',
            'pvt_topic':                   '/gnss_rover/ubx_nav_pvt',
            'stale_timeout_sec':           1.5,
            'refresh_ms':                  80,
        }],
        condition=IfCondition(LaunchConfiguration('enable_sensor_monitor')),
    )

    # ================================================================
    # GPS RTK GUI (레거시, 기본 비활성화)
    # ================================================================
    gps_rtk_gui_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='gps_rtk_gui_node',
        output='screen',
        parameters=[{
            'gps1_label':             'GPS1 LEFT / ROVER',
            'gps1_fix_topic':         '/gnss_rover/fix',
            'gps1_pvt_topic':         '/gnss_rover/ubx_nav_pvt',
            'gps1_relpos_topic':      '/gnss_rover/ubx_nav_rel_pos_ned',
            'gps2_label':             'GPS2 RIGHT / BASE',
            'gps2_fix_topic':         '/gnss_base/fix',
            'gps2_pvt_topic':         '/gnss_base/ubx_nav_pvt',
            'gps2_relpos_topic':      '/gnss_base/ubx_nav_rel_pos_ned',
            'heading_topic':          '/dual_f9p/heading',
            'heading_valid_topic':    '/dual_f9p/heading_valid',
            'heading_accuracy_topic': '/dual_f9p/heading_accuracy_deg',
            'baseline_topic':         '/dual_f9p/baseline_length_m',
            'window_name':            'HENES Dual GPS Heading Monitor',
            'log_to_console':         False,
        }],
        condition=IfCondition(LaunchConfiguration('enable_gps_rtk_gui')),
    )

    return LaunchDescription([
        # 인수
        device_config_file,
        device_family,
        base_namespace,
        rover_namespace,
        base_gps_name,
        rover_gps_name,
        base_device,
        rover_device,
        base_device_serial,
        rover_device_serial,
        base_frame_id,
        rover_frame_id,
        rover_rtcm_topic,
        imu_name,
        imu_device,
        imu_baudrate,
        enable_imu,
        enable_status_info,
        enable_gps_rtk_gui,
        enable_imu_gui,
        enable_sensor_monitor,
        enable_eskf,
        imu_topic,
        tilt_threshold_deg,
        turn_threshold_rad_s,
        fix_velocity_input_topic,
        fix_velocity_output_topic,
        status_params_file,
        heading_params_file,
        # 디바이스 탐색
        device_resolver,
        # 노드
        base_gps_container,
        rover_gps_container,
        dual_heading_node,
        imu_launch,
        fix_velocity_bridge,
        imu_gui_node,
        eskf_node,
        status_info_node,
        sensor_monitor_node,
        gps_rtk_gui_node,
    ])
