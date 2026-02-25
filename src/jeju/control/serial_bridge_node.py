#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, serial, json, threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64


class SerialBridgeNode(Node):
    """ROS 2 ↔ Arduino Serial Bridge Node"""
    
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/henes_arduino')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 1.0)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        timeout = self.get_parameter('timeout').value
        
        # Serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'시리얼 연결 성공: {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'시리얼 연결 실패: {e}')
            raise
        
        # Subscribers (ROS 2 → Arduino)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.kp_sub = self.create_subscription(Float64, 'pid/kp', self.kp_callback, 10)
        self.ki_sub = self.create_subscription(Float64, 'pid/ki', self.ki_callback, 10)
        self.kd_sub = self.create_subscription(Float64, 'pid/kd', self.kd_callback, 10)
        self.neutral_angle_sub = self.create_subscription(Float64, 'steering/neutral_angle', self.neutral_angle_callback, 10)
        self.filter_size_sub = self.create_subscription(Float64, 'filter/size', self.filter_size_callback, 10)
        self.min_sensor_sub = self.create_subscription(Float64, 'steering/min_sensor', self.min_sensor_callback, 10)
        self.max_sensor_sub = self.create_subscription(Float64, 'steering/max_sensor', self.max_sensor_callback, 10)
        
        # Publishers (Arduino → ROS 2)
        self.encoder1_pub = self.create_publisher(Int32, 'encoder1', 10)
        self.encoder2_pub = self.create_publisher(Int32, 'encoder2', 10)
        self.steering_angle_pub = self.create_publisher(Float64, 'steering_angle', 10)
        self.steering_error_pub = self.create_publisher(Float64, 'steering_error', 10)
        self.raw_sensor_pub = self.create_publisher(Float64, 'raw_sensor', 10)
        self.pwm_output_pub = self.create_publisher(Float64, 'pwm_output', 10)
        
        # Read thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info('Serial Bridge Node 시작 완료')
    
    def send_json(self, data):
        """JSON 데이터를 Arduino로 전송"""
        try:
            json_str = json.dumps(data) + '\n'
            self.ser.write(json_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'전송 실패: {e}')
    
    def cmd_vel_callback(self, msg):
        """Twist 메시지를 Arduino로 전송"""
        data = {
            'cmd': 'vel',
            'linear': int(msg.linear.x),
            'angular': int(msg.angular.z)
        }
        self.send_json(data)
    
    def kp_callback(self, msg):
        self.send_json({'cmd': 'kp', 'value': msg.data})
    
    def ki_callback(self, msg):
        self.send_json({'cmd': 'ki', 'value': msg.data})
    
    def kd_callback(self, msg):
        self.send_json({'cmd': 'kd', 'value': msg.data})
    
    def neutral_angle_callback(self, msg):
        self.send_json({'cmd': 'neutral_angle', 'value': int(msg.data)})
    
    def filter_size_callback(self, msg):
        self.send_json({'cmd': 'filter_size', 'value': int(msg.data)})
    
    def min_sensor_callback(self, msg):
        self.send_json({'cmd': 'min_sensor', 'value': int(msg.data)})
    
    def max_sensor_callback(self, msg):
        self.send_json({'cmd': 'max_sensor', 'value': int(msg.data)})
    
    def read_serial(self):
        """Arduino로부터 데이터 수신 (별도 스레드)"""
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        data = json.loads(line)
                        self.process_arduino_data(data)
            except json.JSONDecodeError:
                self.get_logger().warn(f'JSON 파싱 실패: {line}')
            except Exception as e:
                self.get_logger().error(f'수신 오류: {e}')
    
    def process_arduino_data(self, data):
        """Arduino 데이터를 ROS 2 토픽으로 발행"""
        msg_int32 = Int32()
        msg_float64 = Float64()
        
        if 'enc1' in data:
            msg_int32.data = data['enc1']
            self.encoder1_pub.publish(msg_int32)
        
        if 'enc2' in data:
            msg_int32.data = data['enc2']
            self.encoder2_pub.publish(msg_int32)
        
        if 'steer' in data:
            msg_float64.data = float(data['steer'])
            self.steering_angle_pub.publish(msg_float64)
        
        if 'error' in data:
            msg_float64.data = float(data['error'])
            self.steering_error_pub.publish(msg_float64)
        
        if 'raw_sensor' in data:
            msg_float64.data = float(data['raw_sensor'])
            self.raw_sensor_pub.publish(msg_float64)
        
        if 'pwm' in data:
            msg_float64.data = float(data['pwm'])
            self.pwm_output_pub.publish(msg_float64)
    
    def destroy_node(self):
        """노드 종료 시 시리얼 포트 닫기"""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if self.ser.is_open:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
