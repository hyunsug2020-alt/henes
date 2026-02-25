#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jeju'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(include=['control', 'control.*', 'navigation', 'navigation.*', 'sensing', 'sensing.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'arduino'), glob('arduino/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kmu',
    maintainer_email='kmu@todo.todo',
    description='ROS 2 Humble vehicle control for HENES T870',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = control.teleop_node:main',
            'serial_bridge_node = control.serial_bridge_node:main',
            'status_info_node = sensing.status_info_node:main',
            'wheel_odom_node = sensing.wheel_odom_node:main',
            'path_follower_node = navigation.path_follower_node:main',
            'path_maker_node = navigation.path_maker_node:main',
        ],
    },
)
