import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    ntrip_ros_node = Node(
        package='ntrip_ros',
        executable='ntripclient',
        name='ntrip_ros',
        output='screen',
        parameters=[{
            # ublox_dgnss 입력 기본 토픽
            'rtcm_topic': '/ntrip_client/rtcm',
            
            # 국토지리정보원 서버 설정
            'ntrip_server': 'RTS1.ngii.go.kr:2101',
            # 'ntrip_server': 'RTS2.ngii.go.kr:2101',
            
            'ntrip_user': 'kjb121000',
            'ntrip_pass': 'ngii',
            
            'ntrip_stream': 'VRS-RTCM31',
            # 'ntrip_stream': 'VRS-CMRx',
            # 'ntrip_stream': 'RTK-RTCM32',

            # 고정 좌표 대신 실시간 /fix로 GGA를 생성한다.
            'nmea_gga': '',
            'fix_topic': '/fix',
            'require_live_gga': True,
        }]
    )

    return LaunchDescription([
        ntrip_ros_node
    ])
