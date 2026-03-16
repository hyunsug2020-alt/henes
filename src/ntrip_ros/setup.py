from setuptools import setup
import os
from glob import glob

package_name = 'ntrip_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치 경로 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tilk',
    maintainer_email='tilk@tilk.eu',
    description='The ntrip_ros package for ROS 2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntripclient = ntrip_ros.ntripclient:main',
            'gga_updater = ntrip_ros.gga_updater:main',
        ],
    },
)