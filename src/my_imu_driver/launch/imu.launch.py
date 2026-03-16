#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_device = DeclareLaunchArgument('imu_device', default_value='/dev/henes_imu')
    imu_baudrate = DeclareLaunchArgument('imu_baudrate', default_value='921600')

    imu_node = Node(
        package='my_imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_device'),
            'baudrate': ParameterValue(LaunchConfiguration('imu_baudrate'), value_type=int),
        }],
    )

    return LaunchDescription([
        imu_device,
        imu_baudrate,
        imu_node,
    ])
