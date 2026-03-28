#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    sel_arduino = resolve_device_selection(
        config, 'arduino',
        requested_name=LaunchConfiguration('arduino_name').perform(context).strip(),
        requested_path=LaunchConfiguration('arduino_port').perform(context).strip(),
    )
    actions.append(SetLaunchConfiguration('resolved_arduino', sel_arduino['path']))
    print(f'[path_follow] {format_device_resolution(sel_arduino, "Arduino")}')

    sel_imu = resolve_device_selection(
        config, 'imu',
        requested_name=LaunchConfiguration('imu_name').perform(context).strip(),
        requested_path=LaunchConfiguration('imu_device').perform(context).strip(),
    )
    actions.append(SetLaunchConfiguration('resolved_imu', sel_imu['path']))
    print(f'[path_follow] {format_device_resolution(sel_imu, "IMU")}')

    return actions


def _prompt_path_id(context):
    selected = LaunchConfiguration('path_id').perform(context).strip()
    if selected:
        return []

    path_dir_raw = LaunchConfiguration('path_dir').perform(context).strip()
    path_dir = Path(path_dir_raw).expanduser()
    candidates = []
    if path_dir.exists():
        for p in sorted(path_dir.glob('*_filtered.txt')):
            m = re.match(r'^(\d+)_filtered\.txt$', p.name)
            if m:
                candidates.append(m.group(1))

    if not candidates:
        print(f'[path_follow] WARN: no numbered path files under {path_dir}')
        print('[path_follow] hint: run path_make.launch.py first.')
        return []

    print(f'[path_follow] available path IDs in {path_dir}: {", ".join(candidates)}')
    try:
        entered = input('[path_follow] 추종할 경로 번호를 입력하고 Enter: ').strip()
    except EOFError:
        entered = ''
    if not entered:
        entered = candidates[-1]
        print(f'[path_follow] 입력 없음 -> 최신 번호 사용: {entered}')
    return [SetLaunchConfiguration('path_id', entered)]


def generate_launch_description():
    # ── 인수 ──────────────────────────────────────────────────
    enable_serial       = DeclareLaunchArgument('enable_serial',       default_value='false')
    enable_status       = DeclareLaunchArgument('enable_status',       default_value='false')
    enable_imu          = DeclareLaunchArgument('enable_imu',          default_value='false')
    enable_lidar        = DeclareLaunchArgument('enable_lidar',        default_value='false')
    enable_mpc_follower = DeclareLaunchArgument('enable_mpc_follower', default_value='true')
    enable_mpc_logger   = DeclareLaunchArgument('enable_mpc_logger',   default_value='true')
    enable_rviz         = DeclareLaunchArgument('enable_rviz',         default_value='false')
    device_config_file  = DeclareLaunchArgument('device_config_file',  default_value=default_device_config_path())
    arduino_name        = DeclareLaunchArgument('arduino_name',        default_value='')
    arduino_port        = DeclareLaunchArgument('arduino_port',        default_value='')
    imu_name            = DeclareLaunchArgument('imu_name',            default_value='')
    imu_device          = DeclareLaunchArgument('imu_device',          default_value='')
    imu_baudrate        = DeclareLaunchArgument('imu_baudrate',        default_value='921600')
    path_dir  = DeclareLaunchArgument('path_dir',  default_value='~/henes_ws_ros2/paths')
    path_id   = DeclareLaunchArgument(
        'path_id', default_value='',
        description='Path file number (e.g. 1 for 1_filtered.txt). Empty => prompt on launch.',
    )

    jeju_params_file = DeclareLaunchArgument(
        'jeju_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'), 'config', 'params.yaml'
        ]),
    )

    device_resolver = OpaqueFunction(function=_resolve_devices)
    path_selector   = OpaqueFunction(function=_prompt_path_id)

    # ── Arduino 시리얼 브릿지 ─────────────────────────────────
    serial_bridge_node = Node(
        package='jeju',
        executable='serial_bridge_node.py',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'port':            LaunchConfiguration('resolved_arduino'),
            'baud':            57600,
            'timeout':         0.1,
            'rx_watchdog_sec': 3.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_serial')),
    )

    # ── IMU 드라이버 ──────────────────────────────────────────
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

    # ── status_info_node ──────────────────────────────────────
    status_info_node = Node(
        package='jeju',
        executable='status_info_node',
        name='status_info_node',
        output='screen',
        parameters=[LaunchConfiguration('jeju_params_file')],
        condition=IfCondition(LaunchConfiguration('enable_status')),
    )

    # ── MPC 경로 추종 노드 ────────────────────────────────────
    mpc_path_follower_node = Node(
        package='jeju',
        executable='mpc_path_follower_node',
        name='mpc_path_follower_node',
        output='screen',
        parameters=[LaunchConfiguration('jeju_params_file')],
        condition=IfCondition(LaunchConfiguration('enable_mpc_follower')),
    )

    # ── MPC 로거 ─────────────────────────────────────────────
    mpc_logger_node = Node(
        package='jeju',
        executable='mpc_logger_node',
        name='mpc_logger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mpc_logger')),
    )

    # ── 경로 파일 로더 ────────────────────────────────────────
    path_loader_node = Node(
        package='jeju',
        executable='path_file_loader_node.py',
        name='path_file_loader_node',
        output='screen',
        parameters=[{
            'path_dir':        LaunchConfiguration('path_dir'),
            'path_id':         ParameterValue(LaunchConfiguration('path_id'), value_type=str),
            'path_topic':      '/global_path',
            'publish_rate_hz': 1.0,
        }],
    )

    # ── 지나온 경로 기록 ──────────────────────────────────────
    traveled_path_node = Node(
        package='jeju',
        executable='traveled_path_node.py',
        name='traveled_path_node',
        output='screen',
    )

    # ── MPC 모니터 GUI ────────────────────────────────────────
    mpc_monitor_node = Node(
        package='jeju',
        executable='mpc_monitor_node',
        name='mpc_monitor_node',
        output='screen',
    )

    # ── RViz2 ─────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('jeju'), 'config', 'path_follow.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    # ── TF 고정 ───────────────────────────────────────────────
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    return LaunchDescription([
        # 인수
        enable_serial, enable_status, enable_imu, enable_lidar,
        enable_mpc_follower, enable_mpc_logger, enable_rviz,
        device_config_file,
        arduino_name, arduino_port,
        imu_name, imu_device, imu_baudrate,
        path_dir, path_id,
        jeju_params_file,
        # 디바이스 탐색 → 경로 선택
        device_resolver,
        path_selector,
        # 노드
        serial_bridge_node,
        imu_launch,
        status_info_node,
        mpc_path_follower_node,
        mpc_logger_node,
        path_loader_node,
        traveled_path_node,
        mpc_monitor_node,
        rviz_node,
        map_to_odom_tf,
        base_link_tf,
    ])
