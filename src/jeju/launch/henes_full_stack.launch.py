#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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


def _get_udev_property_map(device_path):
    if not device_path:
        return {}

    real_device = os.path.realpath(device_path)
    try:
        result = subprocess.run(
            ['udevadm', 'info', '-q', 'property', '-n', real_device],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return {}

    if result.returncode != 0:
        return {}

    props = {}
    for line in result.stdout.splitlines():
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        props[key.strip()] = value.strip()
    return props


def _resolve_gps_serial(context, gps_family):
    gps_serial = LaunchConfiguration('gps_device_serial').perform(context).strip()
    if gps_serial:
        if gps_family.upper() == 'F9P':
            print(
                '[jeju] WARN: gps_device_serial was set for F9P. '
                'ublox_dgnss may fail to match unreliable USB iSerial on this family.'
            )
        return gps_serial

    if gps_family.upper() == 'F9P':
        gps_device = LaunchConfiguration('resolved_gps_device').perform(context).strip()
        props = _get_udev_property_map(gps_device)
        udev_serial = props.get('ID_SERIAL_SHORT', '').strip()
        if udev_serial:
            print(
                f'[jeju] INFO: F9P udev serial for {gps_device} is {udev_serial}, '
                'but DEVICE_SERIAL_STRING auto-lock is disabled because ublox_dgnss '
                'reports unreliable USB iSerial on this family.'
            )
        return ''

    gps_device = LaunchConfiguration('resolved_gps_device').perform(context).strip()
    props = _get_udev_property_map(gps_device)
    return props.get('ID_SERIAL_SHORT', '').strip()


def _resolve_device_aliases(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    actions = []

    for role, name_key, path_key, resolved_key in (
        ('arduino', 'arduino_name', 'arduino_port', 'resolved_arduino_port'),
        ('imu', 'imu_name', 'imu_device', 'resolved_imu_device'),
        ('gps', 'gps_name', 'gps_device', 'resolved_gps_device'),
    ):
        selection = resolve_device_selection(
            config,
            role,
            requested_name=LaunchConfiguration(name_key).perform(context).strip(),
            requested_path=LaunchConfiguration(path_key).perform(context).strip(),
        )
        actions.append(SetLaunchConfiguration(resolved_key, selection['path']))
        print(f'[jeju] {format_device_resolution(selection, f"{role} device")}')

    return actions


def _build_gps_launch(context):
    enabled = LaunchConfiguration('enable_gps').perform(context).strip().lower()
    if enabled not in ('1', 'true', 'yes', 'on'):
        return []

    gps_launch_file = LaunchConfiguration('gps_launch_file').perform(context).strip()
    gps_namespace = LaunchConfiguration('gps_namespace').perform(context).strip()
    gps_family = LaunchConfiguration('gps_device_family').perform(context).strip()
    gps_serial = _resolve_gps_serial(context, gps_family)

    launch_args = {
        'namespace': gps_namespace,
        'device_family': gps_family,
        'device_actual': LaunchConfiguration('resolved_gps_device'),
        'device_serial_string': gps_serial,
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch_file),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    enable_joy = DeclareLaunchArgument('enable_joy', default_value='true')
    enable_teleop = DeclareLaunchArgument('enable_teleop', default_value='true')
    enable_serial = DeclareLaunchArgument('enable_serial', default_value='true')
    enable_gps = DeclareLaunchArgument('enable_gps', default_value='false')
    enable_status = DeclareLaunchArgument('enable_status', default_value='true')
    enable_wheel_odom = DeclareLaunchArgument('enable_wheel_odom', default_value='false')
    enable_path_maker = DeclareLaunchArgument('enable_path_maker', default_value='false')
    enable_follower = DeclareLaunchArgument('enable_follower', default_value='true')
    enable_mpc_path_maker = DeclareLaunchArgument('enable_mpc_path_maker', default_value='false')
    enable_mpc_follower = DeclareLaunchArgument('enable_mpc_follower', default_value='false')
    enable_imu = DeclareLaunchArgument('enable_imu', default_value='true')
    enable_lidar = DeclareLaunchArgument('enable_lidar', default_value='true')
    enable_ntrip = DeclareLaunchArgument('enable_ntrip', default_value='false')
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    arduino_name = DeclareLaunchArgument('arduino_name', default_value='')
    arduino_port = DeclareLaunchArgument('arduino_port', default_value='')
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
    mpc_path_maker_odom_topic = DeclareLaunchArgument('mpc_path_maker_odom_topic', default_value='/odometry/filtered')

    ouster_params_file = DeclareLaunchArgument(
        'ouster_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'config',
            'ouster_config.yaml',
        ])
    )
    gps_launch_file = DeclareLaunchArgument(
        'gps_launch_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('jeju'),
            'launch',
            'ublox_rover_hpposllh_navsatfix_rtk.launch.py',
        ])
    )
    gps_device_family = DeclareLaunchArgument('gps_device_family', default_value='F9P')
    gps_name = DeclareLaunchArgument('gps_name', default_value='')
    gps_namespace = DeclareLaunchArgument('gps_namespace', default_value='')
    gps_device = DeclareLaunchArgument('gps_device', default_value='')
    gps_device_serial = DeclareLaunchArgument(
        'gps_device_serial',
        default_value='',
        description='Optional ID_SERIAL_SHORT of GPS to lock. Empty means no serial lock.',
    )

    ntrip_server = DeclareLaunchArgument('ntrip_server', default_value='RTS1.ngii.go.kr:2101')
    ntrip_user = DeclareLaunchArgument('ntrip_user', default_value='kjb121000')
    ntrip_pass = DeclareLaunchArgument('ntrip_pass', default_value='ngii')
    ntrip_stream = DeclareLaunchArgument('ntrip_stream', default_value='VRS-RTCM31')
    ntrip_gga = DeclareLaunchArgument('ntrip_gga', default_value='')
    ntrip_require_live_gga = DeclareLaunchArgument(
        'ntrip_require_live_gga',
        default_value='true',
        description='Wait for live GGA before NTRIP connect. Disable only when static ntrip_gga is configured.',
    )
    ntrip_live_gga_timeout_sec = DeclareLaunchArgument(
        'ntrip_live_gga_timeout_sec',
        default_value='5.0',
        description='Live GGA freshness timeout used by ntrip_ros.',
    )
    ntrip_fix_topic = DeclareLaunchArgument('ntrip_fix_topic', default_value='/fix')
    ntrip_topic = DeclareLaunchArgument('ntrip_rtcm_topic', default_value='/ublox_gps/rtcm')
    device_resolver = OpaqueFunction(function=_resolve_device_aliases)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_joy')),
    )

    teleop_node = Node(
        package='jeju',
        executable='teleop_node.py',
        name='teleop_node',
        output='screen',
        parameters=[LaunchConfiguration('jeju_params_file')],
        condition=IfCondition(LaunchConfiguration('enable_teleop')),
    )

    serial_bridge_node = Node(
        package='jeju',
        executable='serial_bridge_node.py',
        name='serial_bridge_node',
        output='screen',
        parameters=[
            LaunchConfiguration('jeju_params_file'),
            {'port': LaunchConfiguration('resolved_arduino_port')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_serial')),
    )

    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
        parameters=[LaunchConfiguration('jeju_params_file')],
        condition=IfCondition(LaunchConfiguration('enable_status')),
    )

    wheel_odom_node = Node(
        package='jeju',
        executable='wheel_odom_node.py',
        name='wheel_odom_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_wheel_odom')),
    )

    path_follower_node = Node(
        package='jeju',
        executable='path_follower_node.py',
        name='path_follower_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_follower')),
    )

    path_maker_node = Node(
        package='jeju',
        executable='path_maker_node.py',
        name='path_maker_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('path_maker_odom_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_path_maker')),
    )

    mpc_path_maker_node = Node(
        package='jeju',
        executable='mpc_path_maker_node',
        name='mpc_path_maker_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('mpc_path_maker_odom_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_mpc_path_maker')),
    )

    mpc_path_follower_node = Node(
        package='jeju',
        executable='mpc_path_follower_node',
        name='mpc_path_follower_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('jeju'), 'config', 'params.yaml'
        ])],
        condition=IfCondition(LaunchConfiguration('enable_mpc_follower')),
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

    gps_launch = OpaqueFunction(function=_build_gps_launch)

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ouster_ros'), 'launch', 'driver.launch.py'])
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('ouster_params_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )

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
            'require_live_gga': ParameterValue(LaunchConfiguration('ntrip_require_live_gga'), value_type=bool),
            'live_gga_timeout_sec': ParameterValue(
                LaunchConfiguration('ntrip_live_gga_timeout_sec'),
                value_type=float,
            ),
        }],
        condition=IfCondition(LaunchConfiguration('enable_ntrip')),
    )

    return LaunchDescription([
        enable_joy,
        enable_teleop,
        enable_serial,
        enable_gps,
        enable_status,
        enable_wheel_odom,
        enable_path_maker,
        enable_follower,
        enable_mpc_path_maker,
        enable_mpc_follower,
        enable_imu,
        enable_lidar,
        enable_ntrip,
        device_config_file,
        arduino_name,
        arduino_port,
        imu_name,
        imu_device,
        imu_baudrate,
        jeju_params_file,
        path_maker_odom_topic,
        mpc_path_maker_odom_topic,
        ouster_params_file,
        gps_launch_file,
        gps_device_family,
        gps_name,
        gps_namespace,
        gps_device,
        gps_device_serial,
        ntrip_server,
        ntrip_user,
        ntrip_pass,
        ntrip_stream,
        ntrip_gga,
        ntrip_require_live_gga,
        ntrip_live_gga_timeout_sec,
        ntrip_fix_topic,
        ntrip_topic,
        device_resolver,
        joy_node,
        teleop_node,
        serial_bridge_node,
        status_info_node,
        wheel_odom_node,
        gps_launch,
        path_maker_node,
        mpc_path_maker_node,
        path_follower_node,
        mpc_path_follower_node,
        imu_launch,
        lidar_launch,
        ntrip_node,
    ])
