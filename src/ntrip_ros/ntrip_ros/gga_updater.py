#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GGAUpdater(Node):
    def __init__(self):
        super().__init__('gga_updater')
        
        # GGA 메시지를 발행할 퍼블리셔 생성 (/gga_topic)
        self.gga_pub = self.create_publisher(String, '/gga_topic', 10)
        
        # ublox_gps 노드가 발행하는 NMEA 메시지를 구독할 서브스크라이버 생성
        self.create_subscription(String, '/ublox_gps/nmea', self.nmea_callback, 10)
        
        self.get_logger().info("GGA Updater node started. Listening for NMEA messages...")

    def nmea_callback(self, msg):
        """
        /ublox_gps/nmea 토픽에서 메시지를 수신했을 때 호출되는 콜백 함수
        """
        # NMEA 메시지 중에서 '$GPGGA'로 시작하는 것만 필터링
        if msg.data.startswith('$GPGGA'):
            # 필터링된 GGA 메시지를 /gga_topic으로 발행
            self.gga_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        updater = GGAUpdater()
        rclpy.spin(updater)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
