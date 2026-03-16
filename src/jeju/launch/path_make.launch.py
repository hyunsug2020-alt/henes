#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

from control.device_config import (
    default_device_config_path,
    format_device_resolution,
    load_device_config,
    resolve_device_selection,
)


def _resolve_imu_device(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    selection = resolve_device_selection(
        config,
        'imu',
        requested_name=LaunchConfiguration('imu_name').perform(context).strip(),
        requested_path=LaunchConfiguration('imu_device').perform(context).strip(),
    )
    print(f'[jeju] {format_device_resolution(selection, "imu device")}')
    return [SetLaunchConfiguration('resolved_imu_device', selection['path'])]


def generate_launch_description():
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    imu_name = DeclareLaunchArgument('imu_name', default_value='')
    imu_device = DeclareLaunchArgument('imu_device', default_value='')
    imu_baudrate = DeclareLaunchArgument('imu_baudrate', default_value='921600')
    jeju_params_file = DeclareLaunchArgument(
        'jeju_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'config',
            'params.yaml',
        ])
    )
    path_maker_odom_topic = DeclareLaunchArgument('path_maker_odom_topic', default_value='/odometry/filtered')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='false')
    gps_rtk_fix_topic = DeclareLaunchArgument('gps_rtk_fix_topic', default_value='/fix')
    gps_rtk_quality_topic = DeclareLaunchArgument('gps_rtk_quality_topic', default_value='/gps/quality')

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_imu_driver'), 'launch', 'imu.launch.py'])
        ),
        launch_arguments={
            'imu_device': LaunchConfiguration('resolved_imu_device'),
            'imu_baudrate': LaunchConfiguration('imu_baudrate'),
        }.items(),
    )

    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
        parameters=[LaunchConfiguration('jeju_params_file')],
    )

    path_maker_node = Node(
        package='jeju',
        executable='path_maker_node.py',
        name='path_maker_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('path_maker_odom_topic'),
        }],
    )

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

    return LaunchDescription([
        device_config_file,
        imu_name,
        imu_device,
        imu_baudrate,
        jeju_params_file,
        path_maker_odom_topic,
        enable_gps_rtk_gui,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        OpaqueFunction(function=_resolve_imu_device),
        imu_launch,
        status_info_node,
        path_maker_node,
        gps_rtk_gui_node,
    ])
