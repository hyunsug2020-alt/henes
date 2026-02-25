#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message
from std_msgs.msg import Header
from base64 import b64encode
from threading import Thread
import time
import socket
import select

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
                # 파라미터 값 가져오기
                ntrip_server = self.node.ntrip_server
                ntrip_stream = self.node.ntrip_stream
                ntrip_user = self.node.ntrip_user
                ntrip_pass = self.node.ntrip_pass
                nmea_gga = self.node.nmea_gga

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
                    f"{nmea_gga}"
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
        self.declare_parameter('nmea_gga', '$GPGGA,000000.00,0000.000,N,00000.000,E,1,00,1.0,0.0,M,0.0,M,,*00')

        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.ntrip_server = self.get_parameter('ntrip_server').value
        self.ntrip_user = self.get_parameter('ntrip_user').value
        self.ntrip_pass = self.get_parameter('ntrip_pass').value
        self.ntrip_stream = self.get_parameter('ntrip_stream').value
        self.nmea_gga = self.get_parameter('nmea_gga').value

        self.get_logger().info(f"NTRIP Client initialized with server: {self.ntrip_server}, stream: {self.ntrip_stream}")
        self.get_logger().info(f"Publishing RTCM messages to: {self.rtcm_topic}")
        self.get_logger().info(f"Using NMEA GGA: {self.nmea_gga}")

        # 퍼블리셔 생성
        self.pub = self.create_publisher(Message, self.rtcm_topic, 100)

        # 연결 스레드 시작
        self.connection = NTRIPConnect(self)
        self.connection.start()

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
