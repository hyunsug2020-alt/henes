#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ntrip_client_node.py
====================
NTRIP 캐스터 → RTCM3 → GPS 시리얼 포트 포워딩 노드

파라미터:
  host         NTRIP 캐스터 주소 (예: rtk.ngii.go.kr)
  port         포트 (default: 2101)
  mountpoint   마운트포인트 (예: VRS-RTCM32)
  username     사용자 ID
  password     비밀번호
  gps_port     RTCM 쓸 GPS 포트 (default: /dev/henes_gps)
  gps_baud     GPS 보드레이트 (default: 460800)
  send_gga     VRS 용 GGA 전송 여부 (default: true)
  gga_interval GGA 전송 주기 [s] (default: 10.0)
  fix_topic    현재 위치 수신 토픽 (default: /gnss_left/fix)
"""

import base64
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

try:
    import serial
except ImportError:
    print('[ntrip_client_node] ERROR: pyserial not installed.')
    raise


def _build_gga(lat: float, lon: float) -> str:
    """현재 위치로 NMEA GGA 문장 생성 (NTRIP VRS용)"""
    if math.isnan(lat) or math.isnan(lon):
        return '$GPGGA,000000.00,0000.0000,N,00000.0000,E,1,08,1.0,0.0,M,0.0,M,,*47\r\n'

    lat_abs = abs(lat)
    lat_deg = int(lat_abs)
    lat_min = (lat_abs - lat_deg) * 60.0
    lat_hemi = 'N' if lat >= 0 else 'S'

    lon_abs = abs(lon)
    lon_deg = int(lon_abs)
    lon_min = (lon_abs - lon_deg) * 60.0
    lon_hemi = 'E' if lon >= 0 else 'W'

    body = (f'GPGGA,{time.strftime("%H%M%S")}.00,'
            f'{lat_deg:02d}{lat_min:07.4f},{lat_hemi},'
            f'{lon_deg:03d}{lon_min:07.4f},{lon_hemi},'
            f'1,08,1.0,0.0,M,0.0,M,,')
    chk = 0
    for c in body:
        chk ^= ord(c)
    return f'${body}*{chk:02X}\r\n'


class NtripClientNode(Node):
    def __init__(self):
        super().__init__('ntrip_client_node')

        self.declare_parameter('host',         '')
        self.declare_parameter('port',         2101)
        self.declare_parameter('mountpoint',   '')
        self.declare_parameter('username',     '')
        self.declare_parameter('password',     '')
        self.declare_parameter('gps_port',     '/dev/henes_gps')
        self.declare_parameter('gps_baud',     460800)
        self.declare_parameter('send_gga',     True)
        self.declare_parameter('gga_interval', 10.0)
        self.declare_parameter('fix_topic',    '/gnss_left/fix')

        self._host       = self.get_parameter('host').get_parameter_value().string_value
        self._port       = self.get_parameter('port').get_parameter_value().integer_value
        self._mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value.lstrip('/')
        self._user       = self.get_parameter('username').get_parameter_value().string_value
        self._pass       = self.get_parameter('password').get_parameter_value().string_value
        self._gps_port   = self.get_parameter('gps_port').get_parameter_value().string_value
        self._gps_baud   = self.get_parameter('gps_baud').get_parameter_value().integer_value
        self._send_gga   = self.get_parameter('send_gga').get_parameter_value().bool_value
        self._gga_iv     = self.get_parameter('gga_interval').get_parameter_value().double_value
        fix_topic        = self.get_parameter('fix_topic').get_parameter_value().string_value

        if not self._host or not self._mountpoint:
            self.get_logger().error('host 또는 mountpoint 파라미터가 비어있음. 노드 종료.')
            raise RuntimeError('ntrip: host/mountpoint required')

        # 현재 GPS 위치 (GGA 전송용)
        self._lat = float('nan')
        self._lon = float('nan')
        self._fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self._fix_cb, 10)

        # GPS 시리얼 포트 열기
        self._ser = serial.Serial(self._gps_port, self._gps_baud, timeout=1.0)
        self.get_logger().info(f'GPS 포트 열림: {self._gps_port} @ {self._gps_baud}')

        self._stop = False
        self._sock = None
        self._last_gga = 0.0
        self._bytes_total = 0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'NTRIP 클라이언트 시작: {self._host}:{self._port}/{self._mountpoint}')

    # ── 콜백 ────────────────────────────────────────────────────

    def _fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude):
            self._lat = msg.latitude
            self._lon = msg.longitude

    # ── NTRIP 연결 / 재연결 루프 ────────────────────────────────

    def _connect(self) -> bool:
        try:
            self._sock = socket.create_connection((self._host, self._port), timeout=10.0)
            cred = base64.b64encode(f'{self._user}:{self._pass}'.encode()).decode()
            request = (
                f'GET /{self._mountpoint} HTTP/1.0\r\n'
                f'Host: {self._host}\r\n'
                f'Authorization: Basic {cred}\r\n'
                f'User-Agent: NTRIP HENESRover/1.0\r\n'
                f'Connection: close\r\n'
                f'\r\n'
            )
            self._sock.sendall(request.encode())

            # 응답 헤더 확인
            resp = b''
            while b'\r\n\r\n' not in resp:
                chunk = self._sock.recv(256)
                if not chunk:
                    break
                resp += chunk
                if len(resp) > 4096:
                    break

            header = resp.decode('ascii', errors='ignore')
            if 'ICY 200 OK' in header or '200 OK' in header:
                self.get_logger().info('NTRIP 연결 성공')
                # 헤더 뒤에 붙은 RTCM 데이터 먼저 처리
                idx = resp.find(b'\r\n\r\n')
                leftover = resp[idx + 4:] if idx >= 0 else b''
                if leftover:
                    self._write_rtcm(leftover)
                return True
            elif '401' in header:
                self.get_logger().error('NTRIP 인증 실패 (401) - username/password 확인')
            elif '404' in header:
                self.get_logger().error(f'NTRIP 마운트포인트 없음 (404): {self._mountpoint}')
            else:
                self.get_logger().error(f'NTRIP 연결 실패: {header[:80]}')
            return False
        except Exception as e:
            self.get_logger().error(f'NTRIP 연결 오류: {e}')
            return False

    def _write_rtcm(self, data: bytes):
        try:
            self._ser.write(data)
            self._bytes_total += len(data)
        except Exception as e:
            self.get_logger().error(f'GPS 시리얼 쓰기 오류: {e}')

    def _send_gga_if_needed(self):
        if not self._send_gga:
            return
        now = time.monotonic()
        if now - self._last_gga < self._gga_iv:
            return
        self._last_gga = now
        gga = _build_gga(self._lat, self._lon)
        try:
            self._sock.sendall(gga.encode())
        except Exception:
            pass

    def _run(self):
        while not self._stop:
            if not self._connect():
                time.sleep(5.0)
                continue

            self._sock.settimeout(5.0)
            log_t = time.monotonic()

            try:
                while not self._stop:
                    self._send_gga_if_needed()
                    try:
                        data = self._sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        self.get_logger().warn('NTRIP 연결 끊김, 재시도...')
                        break
                    self._write_rtcm(data)

                    # 30초마다 수신량 로그
                    if time.monotonic() - log_t > 30.0:
                        self.get_logger().info(
                            f'NTRIP 수신 중: 누적 {self._bytes_total/1024:.1f} KB')
                        log_t = time.monotonic()

            except Exception as e:
                self.get_logger().error(f'NTRIP 수신 오류: {e}')
            finally:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

            if not self._stop:
                time.sleep(3.0)

    def destroy_node(self):
        self._stop = True
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NtripClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
