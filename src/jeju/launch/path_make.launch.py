#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_device = DeclareLaunchArgument('imu_device', default_value='/dev/henes_imu')
    gps_device = DeclareLaunchArgument('gps_device', default_value='/dev/henes_gps')
    gps_device_family = DeclareLaunchArgument('gps_device_family', default_value='F9P')
    gps_device_serial = DeclareLaunchArgument('gps_device_serial', default_value='')
    path_maker_odom_topic = DeclareLaunchArgument('path_maker_odom_topic', default_value='/odometry/filtered')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='true')
    enable_rtk = DeclareLaunchArgument('enable_rtk', default_value='true')
    ntrip_server = DeclareLaunchArgument('ntrip_server', default_value='RTS1.ngii.go.kr:2101')
    ntrip_user = DeclareLaunchArgument('ntrip_user', default_value='kjb121000')
    ntrip_pass = DeclareLaunchArgument('ntrip_pass', default_value='ngii')
    ntrip_stream = DeclareLaunchArgument('ntrip_stream', default_value='VRS-RTCM31')
    ntrip_gga = DeclareLaunchArgument(
        'ntrip_gga',
        default_value='$GPGGA,114101.712,3551.578,N,12829.263,E,1,12,1.0,0.0,M,0.0,M,,*61',
    )
    ntrip_topic = DeclareLaunchArgument('ntrip_rtcm_topic', default_value='/ntrip_client/rtcm')
    gps_rtk_fix_topic = DeclareLaunchArgument('gps_rtk_fix_topic', default_value='/fix')
    gps_rtk_quality_topic = DeclareLaunchArgument('gps_rtk_quality_topic', default_value='/gps/quality')

    gps_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jeju'), 'launch', 'gps_only.launch.py'])
        ),
        launch_arguments={
            'gps_device': LaunchConfiguration('gps_device'),
            'gps_device_family': LaunchConfiguration('gps_device_family'),
            'gps_device_serial': LaunchConfiguration('gps_device_serial'),
            'enable_rtk': LaunchConfiguration('enable_rtk'),
            'ntrip_server': LaunchConfiguration('ntrip_server'),
            'ntrip_user': LaunchConfiguration('ntrip_user'),
            'ntrip_pass': LaunchConfiguration('ntrip_pass'),
            'ntrip_stream': LaunchConfiguration('ntrip_stream'),
            'ntrip_gga': LaunchConfiguration('ntrip_gga'),
            'ntrip_rtcm_topic': LaunchConfiguration('ntrip_rtcm_topic'),
        }.items(),
    )

    imu_node = Node(
        package='my_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_device'),
        }],
    )

    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
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
        imu_device,
        gps_device,
        gps_device_family,
        gps_device_serial,
        path_maker_odom_topic,
        enable_gps_rtk_gui,
        enable_rtk,
        ntrip_server,
        ntrip_user,
        ntrip_pass,
        ntrip_stream,
        ntrip_gga,
        ntrip_topic,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        gps_stack,
        imu_node,
        status_info_node,
        path_maker_node,
        gps_rtk_gui_node,
    ])
