#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, json, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64


class SerialBridgeNode(Node):
    """ROS 2 ↔ Arduino Serial Bridge Node"""

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/henes_arduino')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('rx_watchdog_sec', 0.0)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('max_steer_deg', 55)

        self.port               = str(self.get_parameter('port').value)
        self.baud               = int(self.get_parameter('baud').value)
        self.timeout            = float(self.get_parameter('timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.rx_watchdog_sec    = float(self.get_parameter('rx_watchdog_sec').value)
        self.cmd_topic          = str(self.get_parameter('cmd_topic').value)
        self.max_steer_deg      = int(self.get_parameter('max_steer_deg').value)

        self.ser, self.serial_connected = None, False
        self.ser_lock          = threading.Lock()
        self._last_reconnect   = 0.0
        self._last_connect_err = ''
        self._rx_buffer        = ''
        self._last_rx_time     = time.monotonic()
        self._no_rx_warned     = False

        if not self._connect_serial(initial=True):
            raise RuntimeError(f'시리얼 연결 초기화 실패: {self._last_connect_err}')

        self.cmd_vel_sub        = self.create_subscription(Twist,   self.cmd_topic,           self.cmd_vel_callback,        10)
        self.kp_sub             = self.create_subscription(Float64, 'pid/kp',                 self.kp_callback,             10)
        self.ki_sub             = self.create_subscription(Float64, 'pid/ki',                 self.ki_callback,             10)
        self.kd_sub             = self.create_subscription(Float64, 'pid/kd',                 self.kd_callback,             10)
        self.neutral_angle_sub  = self.create_subscription(Float64, 'steering/neutral_angle', self.neutral_angle_callback,  10)
        self.filter_size_sub    = self.create_subscription(Float64, 'filter/size',            self.filter_size_callback,    10)
        self.min_sensor_sub     = self.create_subscription(Float64, 'steering/min_sensor',    self.min_sensor_callback,     10)
        self.max_sensor_sub     = self.create_subscription(Float64, 'steering/max_sensor',    self.max_sensor_callback,     10)

        self.encoder1_pub       = self.create_publisher(Int32,   'encoder1',           10)
        self.encoder2_pub       = self.create_publisher(Int32,   'encoder2',           10)
        self.steering_angle_pub = self.create_publisher(Float64, 'steering_angle',     10)
        self.steering_error_pub = self.create_publisher(Float64, 'steering_error',     10)
        self.raw_sensor_pub     = self.create_publisher(Float64, 'raw_sensor',         10)
        self.pwm_output_pub     = self.create_publisher(Float64, 'pwm_output',         10)
        self.heartbeat_pub      = self.create_publisher(Int32,   '/arduino_heartbeat', 10)

        self.running     = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        self.health_timer = self.create_timer(1.0, self._health_check)

        self.get_logger().info(f'cmd 구독 토픽: {self.cmd_topic}')
        self.get_logger().info('Serial Bridge Node 시작 완료')

    def _connect_serial(self, initial=False):
        try:
            with self.ser_lock:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
                ser          = serial.Serial()
                ser.port     = self.port
                ser.baudrate = self.baud
                ser.timeout  = self.timeout
                ser.dsrdtr   = False
                ser.rtscts   = False
                ser.dtr      = False   # open() 전 DTR Low → Arduino 자동 리셋 방지
                try:
                    ser.exclusive = True
                except Exception:
                    pass
                ser.open()
                time.sleep(2.5)        # Arduino setup() 완료 대기
                ser.reset_input_buffer()
                self.ser, self.serial_connected = ser, True
                self._last_rx_time = time.monotonic()
            self.get_logger().info(f'시리얼 연결 성공: {self.port} @ {self.baud} baud')
            return True
        except serial.SerialException as e:
            self.serial_connected  = False
            self._last_connect_err = str(e)
            if initial:
                self.get_logger().error(f'시리얼 연결 실패: {e}')
            else:
                self.get_logger().warn(f'시리얼 재연결 실패: {e}')
            return False

    def _try_reconnect(self):
        now = time.monotonic()
        if (now - self._last_reconnect) < self.reconnect_interval:
            return
        self._last_reconnect = now
        self._connect_serial(initial=False)

    def _force_reopen(self, reason: str):
        with self.ser_lock:
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser, self.serial_connected = None, False
        self.get_logger().warn(f'시리얼 재오픈: {reason}')

    def _health_check(self):
        if not self.serial_connected:
            return
        idle = time.monotonic() - self._last_rx_time
        if idle > 3.0 and not self._no_rx_warned:
            self._no_rx_warned = True
            self.get_logger().error(
                f'Arduino 수신 없음 {idle:.1f}s. henes_control_ros2.ino 업로드 및 baud(57600) 확인.')

    def send_json(self, data):
        try:
            if not self.serial_connected:
                self._try_reconnect()
                if not self.serial_connected:
                    return
            msg = json.dumps(data) + '\n'
            with self.ser_lock:
                if self.ser is None or not self.ser.is_open:
                    self.serial_connected = False
                    return
                self.ser.write(msg.encode('utf-8'))
        except serial.SerialException as e:
            self.serial_connected = False
            self.get_logger().error(f'전송 실패: {e}')
        except Exception as e:
            self.get_logger().error(f'전송 오류: {e}')

    def cmd_vel_callback(self, msg):
        steer = int(msg.angular.z)
        if steer > self.max_steer_deg:
            steer = self.max_steer_deg
        elif steer < -self.max_steer_deg:
            steer = -self.max_steer_deg
        self.send_json({'cmd': 'vel', 'linear': int(msg.linear.x), 'angular': steer})

    def kp_callback(self, msg):             self.send_json({'cmd': 'kp',           'value': msg.data})
    def ki_callback(self, msg):             self.send_json({'cmd': 'ki',           'value': msg.data})
    def kd_callback(self, msg):             self.send_json({'cmd': 'kd',           'value': msg.data})
    def neutral_angle_callback(self, msg):  self.send_json({'cmd': 'neutral_angle','value': int(msg.data)})
    def filter_size_callback(self, msg):    self.send_json({'cmd': 'filter_size',  'value': int(msg.data)})
    def min_sensor_callback(self, msg):     self.send_json({'cmd': 'min_sensor',   'value': int(msg.data)})
    def max_sensor_callback(self, msg):     self.send_json({'cmd': 'max_sensor',   'value': int(msg.data)})

    def read_serial(self):
        while self.running and rclpy.ok():
            try:
                if not self.serial_connected:
                    self._try_reconnect()
                    time.sleep(0.05)
                    continue
                with self.ser_lock:
                    if self.ser is None or not self.ser.is_open:
                        self.serial_connected = False
                        continue
                    ser = self.ser
                chunk = ser.read(256)
                if not chunk:
                    if self.rx_watchdog_sec > 0.0:
                        idle = time.monotonic() - self._last_rx_time
                        if idle > self.rx_watchdog_sec:
                            self._force_reopen(f'RX 무수신 {idle:.1f}s')
                    time.sleep(0.005)
                    continue
                self._last_rx_time = time.monotonic()
                self._no_rx_warned = False
                self._rx_buffer   += chunk.decode('utf-8', errors='ignore')
                if len(self._rx_buffer) > 8192:
                    self.get_logger().warn('RX 버퍼 오버플로우, 초기화')
                    self._rx_buffer = ''
                    continue
                while '\n' in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    first = line.find('{')
                    if first > 0:
                        line = line[first:]
                    if not line.startswith('{'):
                        continue
                    last = line.rfind('}')
                    if last >= 0:
                        line = line[:last + 1]
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f'JSON 파싱 실패: {line}')
                        continue
                    if isinstance(data, dict):
                        self.process_arduino_data(data)
            except serial.SerialException as e:
                self.serial_connected = False
                self.get_logger().error(f'수신 실패: {e}')
                time.sleep(0.2)
            except Exception as e:
                self.get_logger().error(f'수신 오류: {e}')
                time.sleep(0.05)

    def process_arduino_data(self, data):
        i32, f64 = Int32(), Float64()
        if 'enc1'       in data: i32.data = int(data['enc1']);        self.encoder1_pub.publish(i32)
        if 'enc2'       in data: i32.data = int(data['enc2']);        self.encoder2_pub.publish(i32)
        if 'steer'      in data: f64.data = float(data['steer']);     self.steering_angle_pub.publish(f64)
        if 'error'      in data: f64.data = float(data['error']);     self.steering_error_pub.publish(f64)
        if 'raw_sensor' in data: f64.data = float(data['raw_sensor']);self.raw_sensor_pub.publish(f64)
        if 'pwm'        in data: f64.data = float(data['pwm']);       self.pwm_output_pub.publish(f64)
        if 'hb'         in data: i32.data = int(data['hb']);          self.heartbeat_pub.publish(i32)

    def destroy_node(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        with self.ser_lock:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                self.get_logger().info('시리얼 포트 닫힘')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
