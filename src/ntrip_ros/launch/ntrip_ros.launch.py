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
            # ublox_gps 드라이버 RTCM 구독 토픽
            'rtcm_topic': '/ublox_gps/rtcm',
            
            # 국토지리정보원 서버 설정
            'ntrip_server': 'RTS1.ngii.go.kr:2101',
            # 'ntrip_server': 'RTS2.ngii.go.kr:2101',
            
            'ntrip_user': 'kjb121000',
            'ntrip_pass': 'ngii',
            
            'ntrip_stream': 'VRS-RTCM31',
            # 'ntrip_stream': 'VRS-CMRx',
            # 'ntrip_stream': 'RTK-RTCM32',
            
            # 공학 4호관 좌표 (VRS 요청용)
            'nmea_gga': '$GPGGA,114101.712,3551.578,N,12829.263,E,1,12,1.0,0.0,M,0.0,M,,*61',
        }]
    )

    return LaunchDescription([
        ntrip_ros_node
    ])
