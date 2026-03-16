from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'moraimpc'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=[
        'sensing', 'sensing.*',
        'navigation', 'navigation.*',
        'parking', 'parking.*',
        'visualization', 'visualization.*',
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='todo@todo.todo',
    description='MORAI MPC - Ioniq 5 SUV',
    license='MIT',
    entry_points={
        'console_scripts': [
            # MORAI 브릿지 (토픽 변환)
            'morai_bridge_node = sensing.morai_bridge_node:main',
            # ESKF (GPS+IMU 헤딩 융합)
            'eskf_node = sensing.eskf_node:main',
            # 주차 매니저 (Hybrid A* + RTI-NMPC)
            'parking_manager_node = parking.parking_manager_node:main',
            # 경로 추종 (path follower)
            'path_follower_node = navigation.path_follower_node:main',
            # Iridescence 통합 GUI
            'iridescence_gui_node = visualization.iridescence_gui_node:main',
        ],
    },
)
