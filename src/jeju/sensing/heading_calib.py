#!/usr/bin/env python3
"""
heading_calib.py
이동 중 GPS 속도 헤딩(절대 기준) vs ESKF 헤딩 실시간 비교
→ 차이값이 yaw_offset_deg 보정값
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64


class HeadingCalib(Node):
    def __init__(self):
        super().__init__('heading_calib')
        self._gps_hdg = None
        self._eskf_hdg = None
        self._speed = 0.0

        self.create_subscription(
            TwistWithCovarianceStamped, '/gnss_left/fix_velocity',
            self._vel_cb, 10)
        self.create_subscription(
            Float64, '/eskf/heading_deg',
            self._eskf_cb, 10)
        self.create_timer(1.0, self._print)
        self.get_logger().info('이동하면서 터미널을 확인하세요.')

    def _vel_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._speed = math.hypot(vx, vy)
        if self._speed > 0.3:
            self._gps_hdg = math.degrees(math.atan2(vy, vx))

    def _eskf_cb(self, msg):
        self._eskf_hdg = msg.data

    def _wrap(self, a):
        while a > 180:  a -= 360
        while a < -180: a += 360
        return a

    def _print(self):
        if self._eskf_hdg is None:
            print('ESKF 헤딩 수신 대기 중...')
            return
        if self._gps_hdg is None or self._speed < 0.3:
            print(f'ESKF={self._eskf_hdg:+7.1f}°  |  GPS: 속도 부족 ({self._speed:.2f} m/s) - 이동하세요')
            return

        diff = self._wrap(self._gps_hdg - self._eskf_hdg)
        new_offset = round(self._wrap(180.0 + diff) / 5) * 5  # 현재 offset=180 기준

        print(
            f'GPS헤딩={self._gps_hdg:+7.1f}°  |  ESKF={self._eskf_hdg:+7.1f}°  |  '
            f'차이={diff:+6.1f}°  |  '
            f'→ yaw_offset_deg 권장값: {new_offset}°'
        )


def main():
    rclpy.init()
    rclpy.spin(HeadingCalib())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
