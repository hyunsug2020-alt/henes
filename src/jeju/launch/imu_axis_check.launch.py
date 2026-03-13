#!/usr/bin/env python3

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


def _resolve_imu_device(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    selection = resolve_device_selection(
        config,
        'imu',
        requested_name=LaunchConfiguration('imu_name').perform(context).strip(),
        requested_path=LaunchConfiguration('imu_device').perform(context).strip(),
    )
    print(f'[jeju] {format_device_resolution(selection, "imu device")}')
    return [SetLaunchConfiguration('resolved_imu_device', selection['path'])]


def generate_launch_description():
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    imu_name = DeclareLaunchArgument('imu_name', default_value='imu1')
    imu_device = DeclareLaunchArgument('imu_device', default_value='')
    imu_baudrate = DeclareLaunchArgument('imu_baudrate', default_value='921600')
    launch_imu_driver = DeclareLaunchArgument('launch_imu_driver', default_value='true')
    imu_topic = DeclareLaunchArgument('imu_topic', default_value='/handsfree/imu')
    tilt_threshold_deg = DeclareLaunchArgument('tilt_threshold_deg', default_value='4.0')
    turn_threshold_rad_s = DeclareLaunchArgument('turn_threshold_rad_s', default_value='0.15')
    refresh_ms = DeclareLaunchArgument('refresh_ms', default_value='80')

    imu_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_imu_driver'), 'launch', 'imu.launch.py'])
        ),
        launch_arguments={
            'imu_device': LaunchConfiguration('resolved_imu_device'),
            'imu_baudrate': LaunchConfiguration('imu_baudrate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_imu_driver')),
    )

    gui_node = Node(
        package='jeju',
        executable='imu_axis_check_gui.py',
        name='imu_axis_check_gui',
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'tilt_threshold_deg': ParameterValue(LaunchConfiguration('tilt_threshold_deg'), value_type=float),
            'turn_threshold_rad_s': ParameterValue(LaunchConfiguration('turn_threshold_rad_s'), value_type=float),
            'refresh_ms': ParameterValue(LaunchConfiguration('refresh_ms'), value_type=int),
        }],
    )

    return LaunchDescription([
        device_config_file,
        imu_name,
        imu_device,
        imu_baudrate,
        launch_imu_driver,
        imu_topic,
        tilt_threshold_deg,
        turn_threshold_rad_s,
        refresh_ms,
        OpaqueFunction(function=_resolve_imu_device),
        imu_driver_launch,
        gui_node,
    ])
