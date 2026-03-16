#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LAUNCH_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(LAUNCH_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(LAUNCH_REPO_ROOT))

from control.device_config import default_device_config_path


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
    enable_serial = DeclareLaunchArgument('enable_serial', default_value='true')
    enable_status = DeclareLaunchArgument('enable_status', default_value='true')
    enable_imu = DeclareLaunchArgument('enable_imu', default_value='true')
    enable_lidar = DeclareLaunchArgument('enable_lidar', default_value='false')
    enable_mpc_follower = DeclareLaunchArgument('enable_mpc_follower', default_value='true')
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    arduino_name = DeclareLaunchArgument('arduino_name', default_value='')
    arduino_port = DeclareLaunchArgument('arduino_port', default_value='')
    imu_name = DeclareLaunchArgument('imu_name', default_value='')
    imu_device = DeclareLaunchArgument('imu_device', default_value='')
    path_dir = DeclareLaunchArgument('path_dir', default_value='~/henes_ws_ros2/paths')
    path_id = DeclareLaunchArgument(
        'path_id',
        default_value='',
        description='Path file number (e.g. 1 for 1_filtered.txt). Empty => prompt on launch.',
    )
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='false')
    gps_rtk_fix_topic = DeclareLaunchArgument('gps_rtk_fix_topic', default_value='/fix')
    gps_rtk_quality_topic = DeclareLaunchArgument('gps_rtk_quality_topic', default_value='/gps/quality')

    gps_rtk_gui_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='gps_rtk_gui_node',
        output='screen',
        parameters=[{
            'fix_topic': LaunchConfiguration('gps_rtk_fix_topic'),
            'quality_topic': LaunchConfiguration('gps_rtk_quality_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_gps_rtk_gui')),
    )
    path_selector = OpaqueFunction(function=_prompt_path_id)
    path_loader_node = Node(
        package='jeju',
        executable='path_file_loader_node',
        name='path_file_loader_node',
        output='screen',
        parameters=[{
            'path_dir': LaunchConfiguration('path_dir'),
            'path_id': LaunchConfiguration('path_id'),
            'path_topic': '/global_path',
        }],
    )
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    base_footprint_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jeju'), 'launch', 'henes_full_stack.launch.py'])
        ),
        launch_arguments={
            'enable_joy': 'false',
            'enable_teleop': 'false',
            'enable_serial': LaunchConfiguration('enable_serial'),
            'enable_gps': 'false',
            'enable_status': LaunchConfiguration('enable_status'),
            'enable_wheel_odom': 'false',
            'enable_path_maker': 'false',
            'enable_follower': 'false',
            'enable_mpc_path_maker': 'false',
            'enable_mpc_follower': LaunchConfiguration('enable_mpc_follower'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'enable_lidar': LaunchConfiguration('enable_lidar'),
            'enable_ntrip': 'false',
            'device_config_file': LaunchConfiguration('device_config_file'),
            'arduino_name': LaunchConfiguration('arduino_name'),
            'arduino_port': LaunchConfiguration('arduino_port'),
            'imu_name': LaunchConfiguration('imu_name'),
            'imu_device': LaunchConfiguration('imu_device'),
        }.items(),
    )

    return LaunchDescription([
        enable_serial,
        enable_status,
        enable_imu,
        enable_lidar,
        enable_mpc_follower,
        device_config_file,
        arduino_name,
        arduino_port,
        imu_name,
        imu_device,
        path_dir,
        path_id,
        enable_gps_rtk_gui,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        path_selector,
        path_loader_node,
        map_to_odom_tf,
        base_footprint_to_base_link_tf,
        stack,
        gps_rtk_gui_node,
    ])
