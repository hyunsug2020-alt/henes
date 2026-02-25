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
    gps_device_serial = DeclareLaunchArgument('gps_device_serial', default_value='')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='true')

    gps_rtk_gui_node = Node(
        package='jeju',
        executable='gps_rtk_gui_node',
        name='gps_rtk_gui_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gps_rtk_gui')),
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('jeju'), 'launch', 'henes_full_stack.launch.py'])
        ),
        launch_arguments={
            'enable_joy': 'false',
            'enable_teleop': 'false',
            'enable_serial': 'true',
            'enable_gps': 'true',
            'enable_status': 'true',
            'enable_wheel_odom': 'false',
            'enable_path_maker': 'false',
            'enable_follower': 'false',
            'enable_mpc_path_maker': 'false',
            'enable_mpc_follower': 'true',
            'enable_imu': 'true',
            'enable_lidar': 'false',
            'enable_ntrip': 'true',
            'imu_device': LaunchConfiguration('imu_device'),
            'gps_device': LaunchConfiguration('gps_device'),
            'gps_device_serial': LaunchConfiguration('gps_device_serial'),
        }.items(),
    )

    return LaunchDescription([
        imu_device,
        gps_device,
        gps_device_serial,
        enable_gps_rtk_gui,
        stack,
        gps_rtk_gui_node,
    ])
