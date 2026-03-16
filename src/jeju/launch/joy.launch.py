#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import stat
import subprocess
import glob
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LAUNCH_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(LAUNCH_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(LAUNCH_REPO_ROOT))

from control.device_config import (
    default_device_config_path,
    format_device_resolution,
    load_device_config,
    resolve_device_selection,
)


def _resolve_arduino_port(context):
    config_file = LaunchConfiguration('device_config_file').perform(context).strip() or default_device_config_path()
    config = load_device_config(config_file)
    selection = resolve_device_selection(
        config,
        'arduino',
        requested_name=LaunchConfiguration('arduino_name').perform(context).strip(),
        requested_path=LaunchConfiguration('arduino_port').perform(context).strip(),
    )
    print(f'[joy] {format_device_resolution(selection, "arduino device")}')
    return [SetLaunchConfiguration('resolved_arduino_port', selection['path'])]


def _preflight(context):
    port = LaunchConfiguration('resolved_arduino_port').perform(context).strip()
    joy_dev = LaunchConfiguration('joy_dev').perform(context).strip()
    control_mode = LaunchConfiguration('control_mode').perform(context).strip().lower()
    launch_actions = []

    if control_mode not in ('json', 'micro_ros'):
        print(f'[joy] WARN: invalid control_mode={control_mode}, fallback to json')
        control_mode = 'json'
        launch_actions.append(SetLaunchConfiguration('control_mode', 'json'))

    if not port:
        print('[joy] WARN: arduino_port is empty.')
        return launch_actions

    if not os.path.exists(port):
        print(f'[joy] ERROR: serial port not found: {port}')
        return launch_actions

    real_port = os.path.realpath(port)
    try:
        # /dev/bus/usb/* is a char device but not a serial tty endpoint.
        if (not stat.S_ISCHR(os.stat(real_port).st_mode)) or (not real_port.startswith('/dev/tty')):
            print(f'[joy] ERROR: serial target is not tty char device: {port} -> {real_port}')
            print('[joy] hint: use arduino_port:=/dev/henes_arduino (or fix udev rule)')
            return launch_actions
    except FileNotFoundError:
        print(f'[joy] ERROR: serial target not found: {port} -> {real_port}')
        return launch_actions

    if not os.access(port, os.R_OK | os.W_OK):
        print(f'[joy] ERROR: no read/write permission on {port}')
        print('[joy] hint: sudo usermod -aG dialout $USER  (then relogin)')
    else:
        print(f'[joy] OK: serial port ready -> {port}')

    # ModemManager may probe ttyACM* and disrupt custom serial protocols.
    mm_state = subprocess.run(
        ['systemctl', 'is-active', 'ModemManager'],
        capture_output=True,
        text=True,
        check=False,
    )
    if mm_state.returncode == 0 and mm_state.stdout.strip() == 'active':
        print('[joy] WARN: ModemManager is active. It can interfere with /dev/ttyACM*.')
        print('[joy] hint: sudo systemctl stop ModemManager')

    if not joy_dev:
        joy_dev = '/dev/input/js0'
    if not os.path.exists(joy_dev):
        joy_candidates = sorted(glob.glob('/dev/input/js*'))
        if joy_candidates:
            print(f'[joy] WARN: requested joy_dev not found: {joy_dev}')
            print(f'[joy] OK: auto-selected joy_dev -> {joy_candidates[0]}')
            launch_actions.append(SetLaunchConfiguration('joy_dev', joy_candidates[0]))
        else:
            print('[joy] ERROR: no joystick device found under /dev/input/js*')
    else:
        print(f'[joy] OK: joystick device ready -> {joy_dev}')

    return launch_actions


def generate_launch_description():
    device_config_file = DeclareLaunchArgument('device_config_file', default_value=default_device_config_path())
    arduino_name = DeclareLaunchArgument('arduino_name', default_value='')
    arduino_port = DeclareLaunchArgument('arduino_port', default_value='')
    joy_dev = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    control_mode = DeclareLaunchArgument('control_mode', default_value='json')
    micro_ros_agent_baud = DeclareLaunchArgument('micro_ros_agent_baud', default_value='115200')
    max_velocity = DeclareLaunchArgument('max_velocity', default_value='80')
    max_steering = DeclareLaunchArgument('max_steering', default_value='80')
    steering_axis = DeclareLaunchArgument('steering_axis', default_value='3')
    manual_button_idx = DeclareLaunchArgument('manual_button_idx', default_value='4')
    auto_button_idx = DeclareLaunchArgument('auto_button_idx', default_value='5')

    port_resolver = OpaqueFunction(function=_resolve_arduino_port)
    preflight = OpaqueFunction(function=_preflight)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_node_json = Node(
        package='jeju',
        executable='teleop_node.py',
        name='teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'max_velocity': ParameterValue(LaunchConfiguration('max_velocity'), value_type=int),
            'max_steering': ParameterValue(LaunchConfiguration('max_steering'), value_type=int),
            'steering_axis': ParameterValue(LaunchConfiguration('steering_axis'), value_type=int),
            'manual_button_idx': ParameterValue(LaunchConfiguration('manual_button_idx'), value_type=int),
            'auto_button_idx': ParameterValue(LaunchConfiguration('auto_button_idx'), value_type=int),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'json'"])),
    )

    teleop_node_micro = Node(
        package='jeju',
        executable='teleop_node.py',
        name='teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/twist_vel',
            'max_velocity': ParameterValue(LaunchConfiguration('max_velocity'), value_type=int),
            'max_steering': ParameterValue(LaunchConfiguration('max_steering'), value_type=int),
            'steering_axis': ParameterValue(LaunchConfiguration('steering_axis'), value_type=int),
            'manual_button_idx': ParameterValue(LaunchConfiguration('manual_button_idx'), value_type=int),
            'auto_button_idx': ParameterValue(LaunchConfiguration('auto_button_idx'), value_type=int),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'micro_ros'"])),
    )

    serial_bridge_node = Node(
        package='jeju',
        executable='serial_bridge_node.py',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('resolved_arduino_port'),
            'baud': 57600,
            'timeout': 0.1,
            # Arduino reset/boot latency can exceed 2.5s right after port open.
            # Keep watchdog disabled in joy standalone launch to avoid reconnect loops.
            'rx_watchdog_sec': 0.0,
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'json'"])),
    )

    micro_ros_agent_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', LaunchConfiguration('resolved_arduino_port'),
            '-b', LaunchConfiguration('micro_ros_agent_baud'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'micro_ros'"])),
    )

    return LaunchDescription([
        device_config_file,
        arduino_name,
        arduino_port,
        joy_dev,
        control_mode,
        micro_ros_agent_baud,
        max_velocity,
        max_steering,
        steering_axis,
        manual_button_idx,
        auto_button_idx,
        port_resolver,
        preflight,
        joy_node,
        teleop_node_json,
        teleop_node_micro,
        serial_bridge_node,
        micro_ros_agent_node,
    ])
