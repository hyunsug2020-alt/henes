#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """HENES T870 조이스틱 제어 런치 파일"""
    
    # 패키지 경로
    pkg_share = get_package_share_directory('jeju')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    # Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    teleop_node = Node(
        package='jeju',
        executable='teleop_node.py',  # .py 추가
        name='teleop_node',
        output='screen',
        parameters=[config_file]
    )
    
    serial_bridge_node = Node(
        package='jeju',
        executable='serial_bridge_node.py',  # .py 추가
        name='serial_bridge_node',
        output='screen',
        parameters=[config_file]
    )
    
    # Static TF publishers (base_link, odom 등)
    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_broadcaster',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        joy_node,
        teleop_node,
        serial_bridge_node,
        static_tf_base_link,
        static_tf_odom,
    ])
