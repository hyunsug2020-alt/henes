#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
import subprocess

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _udev_serial_from_device(device_path: str) -> str:
    if not device_path:
        return ''
    out = subprocess.run(
        ['udevadm', 'info', '-q', 'property', '-n', device_path],
        capture_output=True,
        text=True,
        check=False,
    )
    if out.returncode != 0:
        return ''
    for line in out.stdout.splitlines():
        if line.startswith('ID_SERIAL_SHORT='):
            return line.split('=', 1)[1].strip()
    return ''


def _build_gps_launch(context):
    enabled = LaunchConfiguration('enable_gps').perform(context).strip().lower()
    if enabled not in ('1', 'true', 'yes', 'on'):
        return []

    gps_launch_file = LaunchConfiguration('gps_launch_file').perform(context).strip()
    gps_namespace = LaunchConfiguration('gps_namespace').perform(context).strip()
    gps_device = LaunchConfiguration('gps_device').perform(context).strip()
    gps_serial = LaunchConfiguration('gps_device_serial').perform(context).strip()

    if not gps_serial:
        gps_serial = _udev_serial_from_device(gps_device)

    launch_args = {
        'namespace': gps_namespace,
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
    enable_gps = DeclareLaunchArgument('enable_gps', default_value='true')
    enable_status = DeclareLaunchArgument('enable_status', default_value='true')
    enable_wheel_odom = DeclareLaunchArgument('enable_wheel_odom', default_value='false')
    enable_path_maker = DeclareLaunchArgument('enable_path_maker', default_value='false')
    enable_follower = DeclareLaunchArgument('enable_follower', default_value='true')
    enable_mpc_path_maker = DeclareLaunchArgument('enable_mpc_path_maker', default_value='false')
    enable_mpc_follower = DeclareLaunchArgument('enable_mpc_follower', default_value='false')
    enable_imu = DeclareLaunchArgument('enable_imu', default_value='true')
    enable_lidar = DeclareLaunchArgument('enable_lidar', default_value='true')
    enable_ntrip = DeclareLaunchArgument('enable_ntrip', default_value='true')
    imu_device = DeclareLaunchArgument('imu_device', default_value='/dev/henes_imu')

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
            FindPackageShare('ublox_dgnss'),
            'launch',
            'ublox_x20p_rover_hpposllh_navsatfix.launch.py',
        ])
    )
    gps_namespace = DeclareLaunchArgument('gps_namespace', default_value='')
    gps_device = DeclareLaunchArgument('gps_device', default_value='/dev/henes_gps')
    gps_device_serial = DeclareLaunchArgument(
        'gps_device_serial',
        default_value='',
        description='ID_SERIAL_SHORT of GPS to lock. Empty means resolve from gps_device.',
    )

    ntrip_server = DeclareLaunchArgument('ntrip_server', default_value='RTS1.ngii.go.kr:2101')
    ntrip_user = DeclareLaunchArgument('ntrip_user', default_value='kjb121000')
    ntrip_pass = DeclareLaunchArgument('ntrip_pass', default_value='ngii')
    ntrip_stream = DeclareLaunchArgument('ntrip_stream', default_value='VRS-RTCM31')
    ntrip_gga = DeclareLaunchArgument(
        'ntrip_gga',
        default_value='$GPGGA,114101.712,3551.578,N,12829.263,E,1,12,1.0,0.0,M,0.0,M,,*61',
    )
    ntrip_topic = DeclareLaunchArgument('ntrip_rtcm_topic', default_value='/ublox_gps/rtcm')

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
        parameters=[LaunchConfiguration('jeju_params_file')],
        condition=IfCondition(LaunchConfiguration('enable_serial')),
    )

    status_info_node = Node(
        package='jeju',
        executable='status_info_node.py',
        name='status_info_node',
        output='screen',
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
        condition=IfCondition(LaunchConfiguration('enable_mpc_follower')),
    )

    imu_node = Node(
        package='my_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_device'),
        }],
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
        imu_device,
        jeju_params_file,
        path_maker_odom_topic,
        mpc_path_maker_odom_topic,
        ouster_params_file,
        gps_launch_file,
        gps_namespace,
        gps_device,
        gps_device_serial,
        ntrip_server,
        ntrip_user,
        ntrip_pass,
        ntrip_stream,
        ntrip_gga,
        ntrip_topic,
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
        imu_node,
        lidar_launch,
        ntrip_node,
    ])
