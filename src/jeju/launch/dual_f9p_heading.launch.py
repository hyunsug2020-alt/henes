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


def _resolve_dual_gps_devices(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    actions = []

    for label, name_key, path_key, resolved_key in (
        ('base', 'base_gps_name', 'base_device', 'resolved_base_device'),
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

    print(
        '[jeju] WARN: dual F9P selection is not guaranteed with the current '
        'ublox_dgnss driver path matching. Verify base/rover mapping on hardware.'
    )
    print(
        '[jeju] INFO: dual heading needs GPS2(base) -> GPS1(rover) RTCM on UART2, '
        'or an equivalent RTCM feed into the rover.'
    )
    return actions


def generate_launch_description():
    gps_launch_file = DeclareLaunchArgument(
        'gps_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'launch',
            'ublox_moving_rover_rtk.launch.py',
        ])
    )
    base_gps_launch_file = DeclareLaunchArgument(
        'base_gps_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'launch',
            'ublox_moving_base_rtk.launch.py',
        ])
    )
    rover_gps_launch_file = DeclareLaunchArgument(
        'rover_gps_launch_file',
        default_value=LaunchConfiguration('gps_launch_file'),
    )
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    device_family = DeclareLaunchArgument('device_family', default_value='F9P')
    base_namespace = DeclareLaunchArgument('base_namespace', default_value='gnss_base')
    rover_namespace = DeclareLaunchArgument('rover_namespace', default_value='gnss_rover')
    base_gps_name = DeclareLaunchArgument('base_gps_name', default_value='gps2')
    rover_gps_name = DeclareLaunchArgument('rover_gps_name', default_value='gps1')
    base_device = DeclareLaunchArgument('base_device', default_value='')
    rover_device = DeclareLaunchArgument('rover_device', default_value='')
    base_device_serial = DeclareLaunchArgument('base_device_serial', default_value='')
    rover_device_serial = DeclareLaunchArgument('rover_device_serial', default_value='')
    base_frame_id = DeclareLaunchArgument('base_frame_id', default_value='gnss_base')
    rover_frame_id = DeclareLaunchArgument('rover_frame_id', default_value='gnss_rover')
    rover_rtcm_topic = DeclareLaunchArgument('rover_rtcm_topic', default_value='/gnss_base/rtcm')
    imu_name = DeclareLaunchArgument('imu_name', default_value='imu1')
    imu_device = DeclareLaunchArgument('imu_device', default_value='')
    imu_baudrate = DeclareLaunchArgument('imu_baudrate', default_value='921600')
    enable_imu = DeclareLaunchArgument('enable_imu', default_value='true')
    enable_status_info = DeclareLaunchArgument('enable_status_info', default_value='true')
    status_params_file = DeclareLaunchArgument(
        'status_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'config',
            'params.yaml',
        ])
    )
    fix_velocity_input_topic = DeclareLaunchArgument('fix_velocity_input_topic', default_value='/gnss_rover/ubx_nav_vel_ned')
    fix_velocity_output_topic = DeclareLaunchArgument('fix_velocity_output_topic', default_value='/ublox_gps/fix_velocity')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='true')
    heading_params_file = DeclareLaunchArgument(
        'heading_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'config',
            'dual_f9p_heading.yaml',
        ])
    )
    device_resolver = OpaqueFunction(function=_resolve_dual_gps_devices)

    base_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('base_gps_launch_file')),
        launch_arguments={
            'namespace': LaunchConfiguration('base_namespace'),
            'instance_name': 'base',
            'device_family': LaunchConfiguration('device_family'),
            'device_actual': LaunchConfiguration('resolved_base_device'),
            'device_serial_string': LaunchConfiguration('base_device_serial'),
            'frame_id': LaunchConfiguration('base_frame_id'),
        }.items(),
    )

    rover_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('rover_gps_launch_file')),
        launch_arguments={
            'namespace': LaunchConfiguration('rover_namespace'),
            'instance_name': 'rover',
            'device_family': LaunchConfiguration('device_family'),
            'device_actual': LaunchConfiguration('resolved_rover_device'),
            'device_serial_string': LaunchConfiguration('rover_device_serial'),
            'frame_id': LaunchConfiguration('rover_frame_id'),
            'rtcm_input_topic': LaunchConfiguration('rover_rtcm_topic'),
        }.items(),
    )

    dual_heading_node = Node(
        package='jeju',
        executable='dual_gnss_heading_node.py',
        name='dual_gnss_heading_node',
        output='screen',
        parameters=[LaunchConfiguration('heading_params_file')],
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_imu_driver'), 'launch', 'imu.launch.py'])
        ),
        launch_arguments={
            'imu_device': LaunchConfiguration('resolved_imu_device'),
            'imu_baudrate': LaunchConfiguration('imu_baudrate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    fix_velocity_bridge = Node(
        package='jeju',
        executable='gnss_velocity_bridge_node.py',
        name='gnss_velocity_bridge_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('fix_velocity_input_topic'),
            'output_topic': LaunchConfiguration('fix_velocity_output_topic'),
            'frame_id': 'odom',
        }],
    )

    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
        parameters=[
            LaunchConfiguration('status_params_file'),
            {
                'gps_fix_topic': '/gnss_rover/fix',
                'gps_vel_topic': LaunchConfiguration('fix_velocity_output_topic'),
                'imu_topic': '/handsfree/imu',
                'dual_heading_topic': '/dual_f9p/heading',
                'dual_heading_valid_topic': '/dual_f9p/heading_valid',
            },
        ],
        condition=IfCondition(LaunchConfiguration('enable_status_info')),
    )

    gps_rtk_gui_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='gps_rtk_gui_node',
        output='screen',
        parameters=[{
            'gps1_label': 'GPS1 LEFT / ROVER',
            'gps1_fix_topic': '/gnss_rover/fix',
            'gps1_pvt_topic': '/gnss_rover/ubx_nav_pvt',
            'gps1_relpos_topic': '/gnss_rover/ubx_nav_rel_pos_ned',
            'gps2_label': 'GPS2 RIGHT / BASE',
            'gps2_fix_topic': '/gnss_base/fix',
            'gps2_pvt_topic': '/gnss_base/ubx_nav_pvt',
            'gps2_relpos_topic': '/gnss_base/ubx_nav_rel_pos_ned',
            'heading_topic': '/dual_f9p/heading',
            'heading_valid_topic': '/dual_f9p/heading_valid',
            'heading_accuracy_topic': '/dual_f9p/heading_accuracy_deg',
            'baseline_topic': '/dual_f9p/baseline_length_m',
            'window_name': 'HENES Dual GPS Heading Monitor',
            'log_to_console': False,
        }],
        condition=IfCondition(LaunchConfiguration('enable_gps_rtk_gui')),
    )

    return LaunchDescription([
        gps_launch_file,
        base_gps_launch_file,
        rover_gps_launch_file,
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
        status_params_file,
        fix_velocity_input_topic,
        fix_velocity_output_topic,
        enable_gps_rtk_gui,
        heading_params_file,
        device_resolver,
        base_gps,
        rover_gps,
        dual_heading_node,
        imu_launch,
        fix_velocity_bridge,
        status_info_node,
        gps_rtk_gui_node,
    ])
