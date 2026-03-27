#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rtcm_serial_bridge_node.py
==========================
rtcm_msgs/Message → GPS 시리얼 포트 포워딩
ntrip_ros 패키지와 함께 사용

파라미터:
  rtcm_topic  구독 토픽 (default: /ntrip_client/rtcm)
  gps_port    GPS 시리얼 포트 (default: /dev/henes_gps)
  gps_baud    보드레이트 (default: 460800)
"""

import rclpy
from rclpy.node import Node

try:
    from rtcm_msgs.msg import Message as RtcmMessage
except ImportError:
    print('[rtcm_serial_bridge] ERROR: rtcm_msgs not installed.')
    raise

try:
    import serial
except ImportError:
    print('[rtcm_serial_bridge] ERROR: pyserial not installed.')
    raise


class RtcmSerialBridgeNode(Node):
    def __init__(self):
        super().__init__('rtcm_serial_bridge_node')

        self.declare_parameter('rtcm_topic', '/ntrip_client/rtcm')
        self.declare_parameter('gps_port',   '/dev/henes_gps')
        self.declare_parameter('gps_baud',   460800)

        rtcm_topic = self.get_parameter('rtcm_topic').get_parameter_value().string_value
        gps_port   = self.get_parameter('gps_port').get_parameter_value().string_value
        gps_baud   = self.get_parameter('gps_baud').get_parameter_value().integer_value

        self._ser = serial.Serial(gps_port, gps_baud, timeout=1.0)
        self._count = 0

        self._sub = self.create_subscription(
            RtcmMessage, rtcm_topic, self._rtcm_cb, 10)

        self.get_logger().info(
            f'RTCM bridge: {rtcm_topic} → {gps_port} @ {gps_baud}')

    def _rtcm_cb(self, msg: RtcmMessage):
        try:
            self._ser.write(bytes(msg.message))
            self._count += 1
            if self._count % 50 == 0:
                self.get_logger().info(f'RTCM 전송: {self._count}개')
        except Exception as e:
            self.get_logger().error(f'시리얼 쓰기 오류: {e}')

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtcmSerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
