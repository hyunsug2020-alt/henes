#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gps_launch_file = DeclareLaunchArgument(
        'gps_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'launch',
            'ublox_rover_hpposllh_navsatfix_rtk.launch.py',
        ])
    )
    device_family = DeclareLaunchArgument('device_family', default_value='F9P')
    base_namespace = DeclareLaunchArgument('base_namespace', default_value='gnss_base')
    rover_namespace = DeclareLaunchArgument('rover_namespace', default_value='gnss_rover')
    base_device_serial = DeclareLaunchArgument('base_device_serial', default_value='')
    rover_device_serial = DeclareLaunchArgument('rover_device_serial', default_value='')
    base_frame_id = DeclareLaunchArgument('base_frame_id', default_value='gnss_base')
    rover_frame_id = DeclareLaunchArgument('rover_frame_id', default_value='gnss_rover')
    heading_params_file = DeclareLaunchArgument(
        'heading_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'config',
            'dual_f9p_heading.yaml',
        ])
    )

    base_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('gps_launch_file')),
        launch_arguments={
            'namespace': LaunchConfiguration('base_namespace'),
            'device_family': LaunchConfiguration('device_family'),
            'device_serial_string': LaunchConfiguration('base_device_serial'),
            'frame_id': LaunchConfiguration('base_frame_id'),
        }.items(),
    )

    rover_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('gps_launch_file')),
        launch_arguments={
            'namespace': LaunchConfiguration('rover_namespace'),
            'device_family': LaunchConfiguration('device_family'),
            'device_serial_string': LaunchConfiguration('rover_device_serial'),
            'frame_id': LaunchConfiguration('rover_frame_id'),
        }.items(),
    )

    dual_heading_node = Node(
        package='jeju',
        executable='dual_gnss_heading_node.py',
        name='dual_gnss_heading_node',
        output='screen',
        parameters=[LaunchConfiguration('heading_params_file')],
    )

    return LaunchDescription([
        gps_launch_file,
        device_family,
        base_namespace,
        rover_namespace,
        base_device_serial,
        rover_device_serial,
        base_frame_id,
        rover_frame_id,
        heading_params_file,
        base_gps,
        rover_gps,
        dual_heading_node,
    ])
