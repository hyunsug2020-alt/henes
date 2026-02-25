#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class TeleopNode(Node):
    """조이스틱 텔레옵 노드 - PS5 듀얼센스 제어"""
    
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_publisher = self.create_publisher(Bool, '/teleop_mode', 10)
        
        # Subscriber
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Parameters
        self.declare_parameter('max_velocity', 20)
        self.declare_parameter('max_steering', 80)
        
        self.MAX_Velocity = self.get_parameter('max_velocity').value
        self.MAX_Steering = self.get_parameter('max_steering').value
        
        # State variables
        self.velocity, self.steering = 0, 0
        self.vel, self.steer = 0, 0
        self.teleop = True
        self.move, self.zero2 = False, False
        self.bool_pubmsg = False
        self.last_teleop_state = True
        self.button_previous_state = []
        
        # Main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('=== 조이스틱 텔레옵 시스템 시작 ===')
        self.get_logger().info('R1: 수동 모드, L1: 자율주행 모드')
        self.get_logger().info(f'현재 모드: {"수동 조작" if self.teleop else "자율주행"}')
    
    def joy_callback(self, data):
        """조이스틱 콜백 - PS5 듀얼센스 기준"""
        self.axes = data.axes
        self.button = data.buttons
        
        if not self.button_previous_state:
            self.button_previous_state = list(self.button)
        
        mode_msg = Bool()
        
        # R1 버튼 (수동 모드)
        if self.button[4] == 1:
            self.teleop = True
            self.bool_pubmsg = False
            mode_msg.data = True
            self.mode_publisher.publish(mode_msg)
        
        # L1 버튼 (자율주행 모드)
        elif self.button[5] == 1:
            self.teleop = False
            self.bool_pubmsg = False
            mode_msg.data = False
            self.mode_publisher.publish(mode_msg)
        
        # 왼쪽 아날로그 스틱 - 속도 제어
        if data.axes[1] is not None:
            self.move = True
            self.vel = int(data.axes[1] * self.MAX_Velocity)
        
        # 오른쪽 아날로그 스틱 - 조향 제어
        if data.axes[3] is not None:
            self.zero2 = True
            self.steer = float(data.axes[3] * self.MAX_Steering)
        
        # Triangle 버튼 - 최대속도 증가
        if self.button_previous_state[2] == 0 and self.button[2] == 1:
            if self.MAX_Velocity + 10 < 230:
                self.MAX_Velocity += 10
                self.get_logger().info(f'최대속도 증가: {self.MAX_Velocity}')
            else:
                self.get_logger().warn(f'최대속도 한계: {self.MAX_Velocity}')
        
        # X 버튼 - 최대속도 감소
        if self.button_previous_state[0] == 0 and self.button[0] == 1:
            if self.MAX_Velocity - 10 > 20:
                self.MAX_Velocity -= 10
                self.get_logger().info(f'최대속도 감소: {self.MAX_Velocity}')
            else:
                self.get_logger().warn(f'최소속도 한계: {self.MAX_Velocity}')
        
        self.button_previous_state = list(self.button)
    
    def control_loop(self):
        """메인 제어 루프 - 10Hz"""
        if self.teleop:
            if self.move:
                self.velocity = self.vel
                self.move = False
            
            if self.zero2:
                self.steering = self.steer
                self.zero2 = False
            
            # 속도 제한
            self.velocity = max(min(self.velocity, self.MAX_Velocity), -self.MAX_Velocity)
            self.steering = max(min(self.steering, self.MAX_Steering), -self.MAX_Steering)
            
            # E-stop 플래그
            self.bool_pubmsg = (self.velocity == 0)
            
            # Twist 메시지 발행
            vel_msg = Twist()
            vel_msg.linear.x = float(self.velocity)
            vel_msg.angular.z = float(self.steering)
            self.vel_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
