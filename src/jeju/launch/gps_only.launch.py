#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def _build_gps_launch(context):
    gps_launch_file = LaunchConfiguration('gps_launch_file').perform(context).strip()
    gps_namespace = LaunchConfiguration('gps_namespace').perform(context).strip()
    gps_serial = LaunchConfiguration('gps_device_serial').perform(context).strip()
    gps_family = LaunchConfiguration('gps_device_family').perform(context).strip()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch_file),
            launch_arguments={
                'namespace': gps_namespace,
                'device_family': gps_family,
                'device_serial_string': gps_serial,
            }.items(),
        )
    ]


def generate_launch_description():
    gps_launch_file = DeclareLaunchArgument(
        'gps_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'launch',
            'ublox_rover_hpposllh_navsatfix_rtk.launch.py',
        ])
    )
    gps_device_family = DeclareLaunchArgument('gps_device_family', default_value='F9P')
    gps_namespace = DeclareLaunchArgument('gps_namespace', default_value='')
    gps_device = DeclareLaunchArgument('gps_device', default_value='/dev/henes_gps')
    gps_device_serial = DeclareLaunchArgument(
        'gps_device_serial',
        default_value='',
        description='Optional ID_SERIAL_SHORT of GPS to lock. Empty means no serial lock.',
    )
    enable_rtk = DeclareLaunchArgument('enable_rtk', default_value='true')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='true')
    ntrip_server = DeclareLaunchArgument('ntrip_server', default_value='RTS1.ngii.go.kr:2101')
    ntrip_user = DeclareLaunchArgument('ntrip_user', default_value='kjb121000')
    ntrip_pass = DeclareLaunchArgument('ntrip_pass', default_value='ngii')
    ntrip_stream = DeclareLaunchArgument('ntrip_stream', default_value='VRS-RTCM31')
    ntrip_gga = DeclareLaunchArgument('ntrip_gga', default_value='')
    ntrip_fix_topic = DeclareLaunchArgument('ntrip_fix_topic', default_value='/fix')
    ntrip_topic = DeclareLaunchArgument('ntrip_rtcm_topic', default_value='/ntrip_client/rtcm')
    gps_rtk_fix_topic = DeclareLaunchArgument('gps_rtk_fix_topic', default_value='/fix')
    gps_rtk_quality_topic = DeclareLaunchArgument('gps_rtk_quality_topic', default_value='/gps/quality')
    gps_rtk_pvt_topic = DeclareLaunchArgument('gps_rtk_pvt_topic', default_value='/ubx_nav_pvt')

    gps_launch = OpaqueFunction(function=_build_gps_launch)
    ntrip_node = Node(
        package='ntrip_ros',
        executable='ntripclient',
        name='ntrip_ros',
        output='screen',
        parameters=[{
            'rtcm_topic': LaunchConfiguration('ntrip_rtcm_topic'),
            'ntrip_server': LaunchConfiguration('ntrip_server'),
            'ntrip_user': LaunchConfiguration('ntrip_user'),
            'ntrip_pass': LaunchConfiguration('ntrip_pass'),
            'ntrip_stream': LaunchConfiguration('ntrip_stream'),
            'nmea_gga': LaunchConfiguration('ntrip_gga'),
            'fix_topic': LaunchConfiguration('ntrip_fix_topic'),
            'require_live_gga': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_rtk')),
    )

    gps_rtk_gui_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='gps_rtk_gui_node',
        output='screen',
        parameters=[{
            'fix_topic': LaunchConfiguration('gps_rtk_fix_topic'),
            'quality_topic': LaunchConfiguration('gps_rtk_quality_topic'),
            'pvt_topic': LaunchConfiguration('gps_rtk_pvt_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_gps_rtk_gui')),
    )

    return LaunchDescription([
        gps_launch_file,
        gps_device_family,
        gps_namespace,
        gps_device,
        gps_device_serial,
        enable_rtk,
        enable_gps_rtk_gui,
        ntrip_server,
        ntrip_user,
        ntrip_pass,
        ntrip_stream,
        ntrip_gga,
        ntrip_fix_topic,
        ntrip_topic,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        gps_rtk_pvt_topic,
        gps_launch,
        ntrip_node,
        gps_rtk_gui_node,
    ])
