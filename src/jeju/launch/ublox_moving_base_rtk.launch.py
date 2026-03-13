#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    instance_name = LaunchConfiguration('instance_name')
    device_family = LaunchConfiguration('device_family')
    device_serial_string = LaunchConfiguration('device_serial_string')
    device_actual = LaunchConfiguration('device_actual')
    frame_id = LaunchConfiguration('frame_id')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value=TextSubstitution(text='INFO'))
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='')
    instance_name_arg = DeclareLaunchArgument(
        'instance_name',
        default_value='base',
        description='Unique instance label used to disambiguate container names')
    device_family_arg = DeclareLaunchArgument(
        'device_family', default_value=TextSubstitution(text='F9P'))
    device_serial_string_arg = DeclareLaunchArgument(
        'device_serial_string',
        default_value='',
        description='Serial string of the device to use')
    device_actual_arg = DeclareLaunchArgument(
        'device_actual',
        default_value='',
        description='Actual /dev path of the device to use when supported by the driver')
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gnss_base',
        description='The frame_id to use in header of published messages')

    params = [
        {'DEVICE_FAMILY': device_family},
        {'DEVICE_SERIAL_STRING': device_serial_string},
        {'DEVICE_ACTUAL': device_actual},
        {'FRAME_ID': frame_id},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_RATE_MEAS': 10},
        {'CFG_RATE_NAV': 100},
        # GPS2 acts as the moving base reference and emits RTCM on UART2.
        {'CFG_UART1INPROT_NMEA': False},
        {'CFG_UART1INPROT_RTCM3X': False},
        {'CFG_UART1INPROT_UBX': False},
        {'CFG_UART1OUTPROT_NMEA': False},
        {'CFG_UART1OUTPROT_RTCM3X': False},
        {'CFG_UART1OUTPROT_UBX': False},
        {'CFG_UART2_BAUDRATE': 0x70800},
        {'CFG_UART2INPROT_NMEA': False},
        {'CFG_UART2INPROT_RTCM3X': False},
        {'CFG_UART2INPROT_UBX': False},
        {'CFG_UART2OUTPROT_NMEA': False},
        {'CFG_UART2OUTPROT_RTCM3X': True},
        {'CFG_UART2OUTPROT_UBX': False},
        {'CFG_USBINPROT_NMEA': False},
        {'CFG_USBINPROT_RTCM3X': True},
        {'CFG_USBINPROT_UBX': True},
        {'CFG_USBOUTPROT_RTCM3X': False},
        {'CFG_USBOUTPROT_UBX': True},
        {'CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2': 1},
        {'CFG_MSGOUT_RTCM_3X_TYPE1074_UART2': 1},
        {'CFG_MSGOUT_RTCM_3X_TYPE1084_UART2': 1},
        {'CFG_MSGOUT_RTCM_3X_TYPE1094_UART2': 1},
        {'CFG_MSGOUT_RTCM_3X_TYPE1124_UART2': 1},
        {'CFG_MSGOUT_RTCM_3X_TYPE1230_UART2': 1},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_SIG_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_VELNED_USB': 1},
    ]

    container1 = ComposableNodeContainer(
        name=[instance_name, TextSubstitution(text='_ublox_dgnss_container')],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace=namespace,
                parameters=params,
            )
        ]
    )

    container2 = ComposableNodeContainer(
        name=[instance_name, TextSubstitution(text='_ublox_nav_sat_fix_hp_container')],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp',
                namespace=namespace
            )
        ]
    )

    return launch.LaunchDescription([
        log_level_arg,
        namespace_arg,
        instance_name_arg,
        device_family_arg,
        device_serial_string_arg,
        device_actual_arg,
        frame_id_arg,
        container1,
        container2,
    ])
