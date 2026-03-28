#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gnss_nmea_node.py
=================
NMEA 시리얼 포트 → sensor_msgs/NavSatFix + TwistWithCovarianceStamped 변환 노드

ublox_dgnss_node 가 UBX 바이너리를 못 가져올 때 (UART/ttyUSB 연결) 사용.
$GNGGA → NavSatFix, $GNVTG → TwistWithCovarianceStamped

파라미터:
  port          (default: /dev/henes_gps)
  baud          (default: 38400)
  fix_topic     (default: /gnss_left/fix)
  vel_topic     (default: /gnss_left/fix_velocity)
  frame_id      (default: gnss_left)
  publish_hz    (default: 10.0)  최대 발행 빈도
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Int32, Float64

try:
    import serial
except ImportError:
    print('[gnss_nmea_node] ERROR: pyserial not installed. pip install pyserial')
    raise

try:
    from rtcm_msgs.msg import Message as RtcmMessage
    _RTCM_AVAILABLE = True
except ImportError:
    _RTCM_AVAILABLE = False


def _parse_lat(val: str, hemi: str) -> float:
    """NMEA ddmm.mmmm → decimal degrees"""
    if not val:
        return float('nan')
    d = int(float(val) / 100)
    m = float(val) - d * 100.0
    deg = d + m / 60.0
    return -deg if hemi in ('S', 'W') else deg


def _nmea_checksum_ok(sentence: str) -> bool:
    if '$' not in sentence or '*' not in sentence:
        return False
    inner = sentence[sentence.index('$') + 1: sentence.index('*')]
    expected = sentence[sentence.index('*') + 1: sentence.index('*') + 3].upper()
    chk = 0
    for c in inner:
        chk ^= ord(c)
    return f'{chk:02X}' == expected


def _gga_quality_to_status(q: int):
    """
    GGA quality → NavSatStatus
    0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float, 6=estimated
    """
    s = NavSatStatus()
    s.service = (NavSatStatus.SERVICE_GPS |
                 NavSatStatus.SERVICE_GLONASS |
                 NavSatStatus.SERVICE_GALILEO |
                 NavSatStatus.SERVICE_COMPASS)
    if q == 0:
        s.status = NavSatStatus.STATUS_NO_FIX
    elif q in (4,):
        s.status = NavSatStatus.STATUS_GBAS_FIX   # RTK fixed
    elif q in (2, 5):
        s.status = NavSatStatus.STATUS_SBAS_FIX   # DGPS / RTK float
    else:
        s.status = NavSatStatus.STATUS_FIX
    return s


class GnssNmeaNode(Node):
    def __init__(self):
        super().__init__('gnss_nmea_node')

        self.declare_parameter('port',        '/dev/henes_gps')
        self.declare_parameter('baud',        460800)
        self.declare_parameter('fix_topic',   '/gnss_left/fix')
        self.declare_parameter('vel_topic',   '/gnss_left/fix_velocity')
        self.declare_parameter('frame_id',    'gnss_left')
        self.declare_parameter('publish_hz',  10.0)
        self.declare_parameter('rtcm_topic',  '')  # 비어있으면 RTCM 비활성

        self._port     = self.get_parameter('port').get_parameter_value().string_value
        self._baud     = self.get_parameter('baud').get_parameter_value().integer_value
        fix_topic      = self.get_parameter('fix_topic').get_parameter_value().string_value
        vel_topic      = self.get_parameter('vel_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        min_dt         = 1.0 / max(1.0, self.get_parameter('publish_hz').get_parameter_value().double_value)
        rtcm_topic     = self.get_parameter('rtcm_topic').get_parameter_value().string_value

        self._fix_pub      = self.create_publisher(NavSatFix, fix_topic, 10)
        self._vel_pub      = self.create_publisher(TwistWithCovarianceStamped, vel_topic, 10)
        self._quality_pub  = self.create_publisher(Int32, fix_topic.replace('/fix', '/gga_quality'), 10)
        self._gps_qual_pub = self.create_publisher(Float64, '/gps/quality', 10)

        # RTCM 구독 (같은 시리얼 포트로 씀)
        self._rtcm_sub = None
        if rtcm_topic and _RTCM_AVAILABLE:
            self._rtcm_sub = self.create_subscription(
                RtcmMessage, rtcm_topic, self._rtcm_cb, 10)
            self.get_logger().info(f'RTCM 수신 활성: {rtcm_topic}')

        self._min_dt  = min_dt
        self._last_t  = 0.0

        # 최신 VTG 속도 [m/s] (GGA 발행 시 같이 냄)
        self._speed_mps = 0.0
        self._course_deg = 0.0

        self._ser = None
        self._ser_lock = threading.Lock()
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'GNSS NMEA node: {self._port}@{self._baud} → {fix_topic}')

    def _rtcm_cb(self, msg: 'RtcmMessage'):
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(bytes(msg.message))
                except Exception as e:
                    self.get_logger().error(f'RTCM write error: {e}')

    @staticmethod
    def _ubx_cmd(msg_class: int, msg_id: int, payload: bytes) -> bytes:
        ck_a = ck_b = 0
        for b in bytes([msg_class, msg_id]) + len(payload).to_bytes(2, 'little') + payload:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return (b'\xB5\x62' + bytes([msg_class, msg_id]) +
                len(payload).to_bytes(2, 'little') + payload + bytes([ck_a, ck_b]))

    @staticmethod
    def _ubx_enable_rtcm_input() -> bytes:
        """UBX-CFG-VALSET: CFG-UART1INPROT-RTCM3X = 1 (RAM only)"""
        payload = bytes([0x00, 0x01, 0x00, 0x00,   # version=0, layers=RAM
                         0x04, 0x00, 0x73, 0x10,   # key CFG-UART1INPROT-RTCM3X
                         0x01])                     # value = true
        return GnssNmeaNode._ubx_cmd(0x06, 0x8A, payload)

    @staticmethod
    def _ubx_set_nav_rate(meas_ms: int = 100) -> bytes:
        """UBX-CFG-RATE: 측정 주기 설정 (100ms = 10Hz)"""
        payload = bytes([
            meas_ms & 0xFF, (meas_ms >> 8) & 0xFF,  # measRate (ms)
            0x01, 0x00,                               # navRate = 1
            0x01, 0x00,                               # timeRef = GPS time
        ])
        return GnssNmeaNode._ubx_cmd(0x06, 0x08, payload)

    def _open_serial(self) -> bool:
        try:
            ser = serial.Serial(self._port, self._baud,
                                timeout=1.0, write_timeout=1.0)
            with self._ser_lock:
                self._ser = ser
            self.get_logger().info(f'Opened {self._port}')
            # 측정 주기 10Hz 설정
            try:
                meas_ms = max(50, int(1000.0 / max(1.0, self.get_parameter('publish_hz')
                                                    .get_parameter_value().double_value)))
                ser.write(self._ubx_set_nav_rate(meas_ms))
                self.get_logger().info(f'UBX nav rate set to {meas_ms}ms ({1000//meas_ms}Hz)')
            except Exception as e:
                self.get_logger().warn(f'Failed to set nav rate: {e}')
            # RTCM 입력 활성화 (CFG-UART1INPROT-RTCM3X = 1)
            try:
                ser.write(self._ubx_enable_rtcm_input())
                self.get_logger().info('UBX RTCM input enabled')
            except Exception as e:
                self.get_logger().warn(f'Failed to enable RTCM input: {e}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self._port}: {e}')
            return False

    def _read_loop(self):
        while not self._stop:
            if self._ser is None or not self._ser.is_open:
                if not self._open_serial():
                    time.sleep(3.0)
                    continue
            try:
                with self._ser_lock:
                    ser = self._ser
                line = ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                with self._ser_lock:
                    self._ser = None
                time.sleep(1.0)
                continue

            if not line.startswith('$'):
                continue
            if not _nmea_checksum_ok(line):
                continue

            talker_msg = line[1:].split(',')[0]  # e.g. GNGGA
            msg_type = talker_msg[2:]             # e.g. GGA

            if msg_type == 'GGA':
                self._handle_gga(line)
            elif msg_type == 'VTG':
                self._handle_vtg(line)

    def _handle_gga(self, line: str):
        # $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        parts = line.split(',')
        if len(parts) < 10:
            return
        try:
            lat     = _parse_lat(parts[2], parts[3])
            lon     = _parse_lat(parts[4], parts[5])
            quality = int(parts[6]) if parts[6] else 0
            num_sv  = int(parts[7]) if parts[7] else 0
            hdop    = float(parts[8]) if parts[8] else 99.0
            alt_str = parts[9]
            alt     = float(alt_str) if alt_str else 0.0
        except (ValueError, IndexError):
            return

        if math.isnan(lat) or math.isnan(lon):
            return

        now = time.monotonic()
        if now - self._last_t < self._min_dt:
            return
        self._last_t = now

        stamp = self.get_clock().now().to_msg()

        # NavSatFix
        fix = NavSatFix()
        fix.header.stamp    = stamp
        fix.header.frame_id = self._frame_id
        fix.status          = _gga_quality_to_status(quality)
        fix.latitude        = lat
        fix.longitude       = lon
        fix.altitude        = alt
        # quality별 base sigma
        if quality == 4:      # RTK Fixed
            sigma_h = 0.02
            sigma_v = 0.05
        elif quality == 5:    # RTK Float
            sigma_h = hdop * 0.3
            sigma_v = hdop * 0.6
        elif quality == 2:    # DGPS
            sigma_h = hdop * 0.5
            sigma_v = hdop * 1.0
        else:                 # standalone
            sigma_h = hdop * 2.5
            sigma_v = hdop * 3.75
        fix.position_covariance[0] = sigma_h ** 2
        fix.position_covariance[4] = sigma_h ** 2
        fix.position_covariance[8] = sigma_v ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._fix_pub.publish(fix)
        q_msg = Int32(); q_msg.data = quality
        self._quality_pub.publish(q_msg)
        # /gps/quality: Float64 0~1 (path maker용)
        _Q = {4: 1.0, 5: 0.85, 2: 0.6, 1: 0.4, 0: 0.0}
        gq = Float64(); gq.data = _Q.get(quality, 0.0)
        self._gps_qual_pub.publish(gq)

        # 속도 (VTG 에서 가져온 값)
        vel = TwistWithCovarianceStamped()
        vel.header.stamp    = stamp
        vel.header.frame_id = self._frame_id
        spd = self._speed_mps
        # VTG True course: 0°=North, 90°=East (clockwise from North)
        # ENU frame: x=East, y=North
        # vx_ENU = sin(course), vy_ENU = cos(course)
        crs = math.radians(self._course_deg)
        vel.twist.twist.linear.x = spd * math.sin(crs)
        vel.twist.twist.linear.y = spd * math.cos(crs)
        self._vel_pub.publish(vel)

    def _handle_vtg(self, line: str):
        # $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,a*hh
        parts = line.split(',')
        if len(parts) < 8:
            return
        try:
            course_t = float(parts[1]) if parts[1] else 0.0
            spd_kmh  = float(parts[7]) if parts[7] else 0.0
            self._course_deg = course_t
            self._speed_mps  = spd_kmh / 3.6
        except ValueError:
            pass

    def destroy_node(self):
        self._stop = True
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GnssNmeaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
