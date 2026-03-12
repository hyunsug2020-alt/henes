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
            'imu_device': LaunchConfiguration('imu_device'),
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
        imu_device,
        imu_baudrate,
        jeju_params_file,
        path_maker_odom_topic,
        enable_gps_rtk_gui,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        imu_launch,
        status_info_node,
        path_maker_node,
        gps_rtk_gui_node,
    ])
