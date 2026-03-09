#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_device = DeclareLaunchArgument('imu_device', default_value='/dev/henes_imu')
    path_maker_odom_topic = DeclareLaunchArgument('path_maker_odom_topic', default_value='/odometry/filtered')
    enable_gps_rtk_gui = DeclareLaunchArgument('enable_gps_rtk_gui', default_value='true')
    gps_rtk_fix_topic = DeclareLaunchArgument('gps_rtk_fix_topic', default_value='/fix')
    gps_rtk_quality_topic = DeclareLaunchArgument('gps_rtk_quality_topic', default_value='/gps/quality')

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
        path_maker_odom_topic,
        enable_gps_rtk_gui,
        gps_rtk_fix_topic,
        gps_rtk_quality_topic,
        imu_node,
        status_info_node,
        path_maker_node,
        gps_rtk_gui_node,
    ])
