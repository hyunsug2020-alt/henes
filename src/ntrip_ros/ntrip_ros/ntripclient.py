#!/usr/bin/env python3

from base64 import b64encode
import math
import select
import socket
from threading import Lock, Thread
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header, String


def _nmea_checksum(payload: str) -> str:
    checksum = 0
    for ch in payload.encode('ascii', errors='ignore'):
        checksum ^= ch
    return f'{checksum:02X}'


def _format_nmea_coordinate(value: float, is_latitude: bool) -> str:
    abs_value = abs(value)
    degrees = int(abs_value)
    minutes = (abs_value - degrees) * 60.0
    if is_latitude:
        return f'{degrees:02d}{minutes:07.4f}'
    return f'{degrees:03d}{minutes:07.4f}'

class NTRIPConnect(Thread):
    def __init__(self, node):
        super(NTRIPConnect, self).__init__()
        self.node = node  # ROS 2 Node 객체 참조
        self.stop = False
        self.retry_count = 0
        self.max_retries = 5
        self.backoff_time = 1

    def run(self):
        while not self.stop and rclpy.ok():
            try:
                if self.node.require_live_gga and not self.node.has_live_gga():
                    self.node.log_waiting_for_live_gga()
                    time.sleep(1.0)
                    continue

                # 파라미터 값 가져오기
                ntrip_server = self.node.ntrip_server
                ntrip_stream = self.node.ntrip_stream
                ntrip_user = self.node.ntrip_user
                ntrip_pass = self.node.ntrip_pass
                nmea_gga = self.node.get_nmea_gga()

                if not ntrip_server or not ntrip_stream:
                    self.node.get_logger().error("NTRIP server/stream is empty")
                    time.sleep(1.0)
                    continue
                if not nmea_gga:
                    self.node.get_logger().warn("No usable NMEA GGA is available yet")
                    time.sleep(1.0)
                    continue

                server_parts = ntrip_server.split(':')
                server = server_parts[0]
                port = int(server_parts[1]) if len(server_parts) > 1 else 2101
                
                self.node.get_logger().info(f"Creating socket connection to {server}:{port}")
                
                # 소켓 생성
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)
                    self.node.get_logger().info("Increased socket buffer size")
                except Exception as e:
                    self.node.get_logger().warn(f"Could not set socket options: {e}")
                
                # 연결
                sock.connect((server, port))
                self.node.get_logger().info("Socket connected")
                
                # 인증 헤더 생성
                auth = b64encode((ntrip_user + ':' + str(ntrip_pass)).encode()).decode()
                
                # HTTP 요청 생성
                request = (
                    f"GET /{ntrip_stream} HTTP/1.1\r\n"
                    f"Host: {server}:{port}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: NTRIP ntrip_ros\r\n"
                    f"Connection: close\r\n"
                    f"Authorization: Basic {auth}\r\n"
                    f"\r\n"
                    f"{nmea_gga.rstrip()}\r\n"
                )
                
                # 요청 전송
                self.node.get_logger().info(f"Sending request to {server}:{port}/{ntrip_stream}")
                sock.send(request.encode())
                self.node.get_logger().info("Request sent")
                
                # 헤더 읽기
                header_data = b""
                start_time = time.time()
                
                while b"\r\n\r\n" not in header_data and time.time() - start_time < 5:
                    if self.stop: break
                    try:
                        ready = select.select([sock], [], [], 1)
                        if ready[0]:
                            chunk = sock.recv(1024)
                            if not chunk:
                                break
                            header_data += chunk
                        else:
                            # 타임아웃이지만 계속 시도
                            continue
                    except socket.timeout:
                        self.node.get_logger().warn("Socket timeout while reading header")
                        break
                    except Exception as e:
                        self.node.get_logger().warn(f"Error reading header: {e}")
                        break
                
                if not header_data:
                    self.node.get_logger().error("No header data received or connection closed")
                    sock.close()
                    time.sleep(2)
                    continue
                
                # 응답 코드 확인
                header_end = header_data.find(b"\r\n\r\n")
                if header_end > 0:
                    header_lines = header_data[:header_end].decode('utf-8', errors='ignore').split("\r\n")
                    status_line = header_lines[0]
                    self.node.get_logger().info(f"Response: {status_line}")
                    
                    if "200 OK" not in status_line:
                        self.node.get_logger().error(f"Failed to connect: {status_line}")
                        sock.close()
                        time.sleep(2)
                        continue
                    
                    # 헤더 후의 데이터 추출
                    initial_data = header_data[header_end + 4:]
                    
                    # RTCM 데이터 읽기
                    sock.settimeout(5)
                    
                    if initial_data:
                        self.process_rtcm_data(initial_data)
                    
                    no_data_count = 0
                    while not self.stop and no_data_count < 10 and rclpy.ok():
                        try:
                            data = sock.recv(1024)
                            if data:
                                no_data_count = 0
                                self.process_rtcm_data(data)
                            else:
                                no_data_count += 1
                                self.node.get_logger().warn(f"Empty data received ({no_data_count}/10)")
                                time.sleep(0.1)
                        except socket.timeout:
                            # 타임아웃은 정상, 다시 루프
                            continue
                        except Exception as e:
                            self.node.get_logger().error(f"Error reading data: {e}")
                            break
                    
                    sock.close()
                    self.node.get_logger().info("Connection closed, reconnecting")
                else:
                    self.node.get_logger().error("Invalid HTTP response header")
                    sock.close()
            
            except socket.timeout:
                self.node.get_logger().warn("Connection timeout")
            except socket.error as e:
                self.node.get_logger().error(f"Socket error: {e}")
            except Exception as e:
                self.node.get_logger().error(f"General error: {type(e).__name__}: {e}")
            
            if self.stop: break

            # 재연결 로직
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.retry_count = 0
                self.backoff_time = 1
                self.node.get_logger().warn("Maximum retries exceeded, resetting")
            else:
                wait_time = min(30, self.backoff_time)
                self.node.get_logger().warn(f"Retry {self.retry_count}/{self.max_retries}, waiting {wait_time} seconds")
                time.sleep(wait_time)
                self.backoff_time *= 1.5

    def process_rtcm_data(self, data):
        if not data:
            return 0
        
        i = 0
        rtcm_msgs_found = 0
        
        while i < len(data):
            if i < len(data) and data[i] == 211: # 0xD3
                if i + 3 < len(data):
                    length = (data[i+1] << 8) + data[i+2]
                    length &= 0x3FF
                    
                    if i + 5 < len(data):
                        # msg_type = ((data[i+3] << 8) + data[i+4]) >> 4
                        total_length = 3 + length + 3
                        
                        if i + total_length <= len(data):
                            rtcm_msg = data[i:i+total_length]
                            
                            rmsg = Message()
                            rmsg.header = Header()
                            # ROS 2 Time 사용
                            rmsg.header.stamp = self.node.get_clock().now().to_msg()
                            rmsg.message = bytes(rtcm_msg)
                            
                            self.node.pub.publish(rmsg)
                            rtcm_msgs_found += 1
                            
                            i += total_length
                            continue
                        else:
                            # 데이터가 잘렸을 경우 경고 (버퍼링 로직이 추가되면 좋음)
                            # self.node.get_logger().warn(f"Incomplete RTCM message")
                            pass
            i += 1
        return rtcm_msgs_found


class NTRIPClient(Node):
    def __init__(self):
        super().__init__('ntripclient')

        # 파라미터 선언 및 가져오기 (ROS 2 방식)
        self.declare_parameter('rtcm_topic', '/ublox_gps/rtcm')
        self.declare_parameter('ntrip_server', '')
        self.declare_parameter('ntrip_user', '')
        self.declare_parameter('ntrip_pass', '')
        self.declare_parameter('ntrip_stream', '')
        self.declare_parameter('nmea_gga', '')
        self.declare_parameter('nmea_gga_topic', '')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('require_live_gga', True)
        self.declare_parameter('live_gga_timeout_sec', 5.0)

        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.ntrip_server = self.get_parameter('ntrip_server').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_pass = self.get_parameter('ntrip_pass').value
        self.ntrip_stream = self.get_parameter('ntrip_stream').value
        self.nmea_gga = self.get_parameter('nmea_gga').value
        self.nmea_gga_topic = self.get_parameter('nmea_gga_topic').value
        self.fix_topic = self.get_parameter('fix_topic').value
        self.require_live_gga = self.get_parameter('require_live_gga').value
        self.live_gga_timeout_sec = float(self.get_parameter('live_gga_timeout_sec').value)

        self._gga_lock = Lock()
        self._live_nmea_gga = ''
        self._live_nmea_gga_time = 0.0
        self._generated_gga = ''
        self._generated_gga_time = 0.0
        self._waiting_for_live_gga_logged = False

        self.get_logger().info(f"NTRIP Client initialized with server: {self.ntrip_server}, stream: {self.ntrip_stream}")
        self.get_logger().info(f"Publishing RTCM messages to: {self.rtcm_topic}")
        if self.nmea_gga:
            self.get_logger().info("Static fallback NMEA GGA is configured")
        if self.nmea_gga_topic:
            self.get_logger().info(f"Live NMEA GGA topic: {self.nmea_gga_topic}")
            self.create_subscription(
                String, self.nmea_gga_topic, self.nmea_callback, qos_profile_sensor_data)
        if self.fix_topic:
            self.get_logger().info(f"NavSatFix topic for dynamic GGA: {self.fix_topic}")
            self.create_subscription(
                NavSatFix, self.fix_topic, self.fix_callback, qos_profile_sensor_data)
        if self.require_live_gga:
            self.get_logger().info("NTRIP connect will wait for a live GPS-based GGA")

        # 퍼블리셔 생성
        self.pub = self.create_publisher(Message, self.rtcm_topic, 100)

        # 연결 스레드 시작
        self.connection = NTRIPConnect(self)
        self.connection.start()

    def _is_recent(self, timestamp: float) -> bool:
        return timestamp > 0.0 and (time.time() - timestamp) <= self.live_gga_timeout_sec

    def has_live_gga(self) -> bool:
        with self._gga_lock:
            return self._is_recent(self._live_nmea_gga_time) or self._is_recent(self._generated_gga_time)

    def get_nmea_gga(self) -> str:
        with self._gga_lock:
            if self._is_recent(self._live_nmea_gga_time):
                return self._live_nmea_gga
            if self._is_recent(self._generated_gga_time):
                return self._generated_gga
            return self.nmea_gga

    def log_waiting_for_live_gga(self) -> None:
        if self._waiting_for_live_gga_logged:
            return
        sources = []
        if self.nmea_gga_topic:
            sources.append(self.nmea_gga_topic)
        if self.fix_topic:
            sources.append(self.fix_topic)
        joined = ', '.join(sources) if sources else 'configured GPS topics'
        self.get_logger().info(f"Waiting for live GGA from {joined} before connecting to NTRIP caster")
        self._waiting_for_live_gga_logged = True

    def nmea_callback(self, msg: String) -> None:
        line = msg.data.strip()
        if not (line.startswith('$GPGGA') or line.startswith('$GNGGA')):
            return
        with self._gga_lock:
            self._live_nmea_gga = line
            self._live_nmea_gga_time = time.time()
        self._waiting_for_live_gga_logged = False

    def fix_callback(self, msg: NavSatFix) -> None:
        gga = self._build_gga_from_fix(msg)
        if not gga:
            return
        with self._gga_lock:
            self._generated_gga = gga
            self._generated_gga_time = time.time()
        self._waiting_for_live_gga_logged = False

    def _build_gga_from_fix(self, msg: NavSatFix) -> str:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude) if math.isfinite(msg.altitude) else 0.0

        if not math.isfinite(lat) or not math.isfinite(lon):
            return ''
        if abs(lat) > 90.0 or abs(lon) > 180.0:
            return ''
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return ''
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return ''

        if msg.status.status >= NavSatStatus.STATUS_SBAS_FIX:
            quality = 2
        else:
            quality = 1

        utc_now = time.gmtime()
        utc_time = time.strftime('%H%M%S', utc_now) + '.00'
        lat_dir = 'N' if lat >= 0.0 else 'S'
        lon_dir = 'E' if lon >= 0.0 else 'W'
        lat_field = _format_nmea_coordinate(lat, is_latitude=True)
        lon_field = _format_nmea_coordinate(lon, is_latitude=False)

        payload = (
            f'GPGGA,{utc_time},{lat_field},{lat_dir},{lon_field},{lon_dir},'
            f'{quality},00,1.0,{alt:.1f},M,0.0,M,,'
        )
        return f'${payload}*{_nmea_checksum(payload)}'

    def stop_connection(self):
        if self.connection is not None:
            self.connection.stop = True
            self.connection.join(timeout=5.0)

def main(args=None):
    rclpy.init(args=args)
    client = None
    try:
        client = NTRIPClient()
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        if client:
            client.stop_connection()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
