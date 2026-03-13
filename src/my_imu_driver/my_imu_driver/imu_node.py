#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import math
import serial.tools.list_ports
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64

# ROS 2에서는 tf.transformations 대신 transforms3d나 numpy를 쓰지만, 
# 간단한 변환을 위해 수식으로 직접 구현하거나 기본적인 쿼터니언 변환 함수를 사용합니다.
# 여기서는 간단히 오일러->쿼터니언 변환을 직접 구현합니다.

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class HandsFreeIMU(Node):
    def __init__(self):
        super().__init__('handsfree_imu_node')
        
        # 파라미터 선언 (기본값 설정)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 퍼블리셔 생성
        self.imu_pub = self.create_publisher(Imu, '/handsfree/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/handsfree/mag', 10)
        
        # 시리얼 연결 시도
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            self.get_logger().info(f"Connected to IMU on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to IMU: {e}")
            return

        # 데이터 처리를 위한 변수들
        self.buff = bytearray()
        self.key = 0
        self.pub_flag = [True, True] # [acc/gyro, mag]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]

        # 타이머 생성 (0.005초마다 실행 = 200Hz)
        self.create_timer(0.005, self.timer_callback)

    def check_sum(self, list_data, check_data):
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xFF) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

    def hex_to_ieee(self, raw_data):
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = bytes(raw_data[i:i+4]).hex()
            ieee_data.append(struct.unpack(">f", bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data

    def handle_serial_data(self, raw_data):
        self.buff.append(raw_data)
        self.key += 1
        
        if self.buff[0] != 0xAA:
            self.buff = bytearray()
            self.key = 0
            return

        if self.key < 3:
            return
            
        if self.buff[1] != 0x55:
            self.buff = bytearray()
            self.key = 0
            return
            
        if self.key < self.buff[2] + 5:
            return
            
        # 데이터 파싱 시작
        data_buff = list(self.buff)
        if self.buff[2] == 0x2C and self.pub_flag[0]:
            if self.check_sum(data_buff[2:47], data_buff[47:49]):
                data = self.hex_to_ieee(data_buff[7:47])
                self.angular_velocity = data[1:4]
                self.acceleration = data[4:7]
                self.magnetometer = data[7:10]
            self.pub_flag[0] = False
            
        elif self.buff[2] == 0x14 and self.pub_flag[1]:
            if self.check_sum(data_buff[2:23], data_buff[23:25]):
                data = self.hex_to_ieee(data_buff[7:23])
                self.angle_degree = data[1:4]
            self.pub_flag[1] = False

        self.buff = bytearray()
        self.key = 0
        self.pub_flag = [True, True]
        
        # 메시지 발행
        self.publish_imu()

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # 각도 변환 (Degree -> Radian)
        roll = self.angle_degree[0] * math.pi / 180
        pitch = self.angle_degree[1] * math.pi / 180
        yaw = self.angle_degree[2] * math.pi / 180

        # Field test matched left/right and yaw, but front/rear tilt was inverted.
        q = quaternion_from_euler(roll, pitch, -yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]

        imu_msg.linear_acceleration.x = self.acceleration[0] * 9.8
        imu_msg.linear_acceleration.y = self.acceleration[1] * 9.8
        imu_msg.linear_acceleration.z = self.acceleration[2] * 9.8

        self.imu_pub.publish(imu_msg)

    def timer_callback(self):
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            for byte in data:
                self.handle_serial_data(byte)

def main(args=None):
    rclpy.init(args=args)
    node = HandsFreeIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
