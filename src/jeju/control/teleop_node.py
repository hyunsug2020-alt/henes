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
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('legacy_cmd_topic', '/twist_vel')
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.legacy_cmd_topic = str(self.get_parameter('legacy_cmd_topic').value)
        self.vel_publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.legacy_vel_publisher = None
        if self.legacy_cmd_topic and self.legacy_cmd_topic != self.cmd_topic:
            self.legacy_vel_publisher = self.create_publisher(Twist, self.legacy_cmd_topic, 10)
        self.mode_publisher = self.create_publisher(Bool, '/teleop_mode', 10)
        
        # Subscriber
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Parameters
        self.declare_parameter('max_velocity', 80)
        self.declare_parameter('max_steering', 80)
        self.declare_parameter('steering_axis', 3)
        self.declare_parameter('manual_button_idx', 4)  # Original behavior
        self.declare_parameter('auto_button_idx', 5)    # Original behavior
        
        self.MAX_Velocity = int(self.get_parameter('max_velocity').value)
        self.MAX_Steering = int(self.get_parameter('max_steering').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.manual_button_idx = int(self.get_parameter('manual_button_idx').value)
        self.auto_button_idx = int(self.get_parameter('auto_button_idx').value)
        
        # State variables
        self.velocity, self.steering = 0, 0
        self.vel, self.steer = 0, 0
        self.teleop = True
        self.move, self.zero2 = False, False
        self.bool_pubmsg = False
        self.last_teleop_state = True
        self.button_previous_state = []
        self.mode_msg = Bool()
        self.mode_msg.data = self.teleop
        
        # Main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('=== 조이스틱 텔레옵 시스템 시작 ===')
        self.get_logger().info(
            f'수동 모드 버튼 idx={self.manual_button_idx}, 자율 모드 버튼 idx={self.auto_button_idx}'
        )
        if self.legacy_vel_publisher is None:
            self.get_logger().info(f'명령 토픽: {self.cmd_topic}')
        else:
            self.get_logger().info(
                f'명령 토픽: {self.cmd_topic} (legacy 동시 발행: {self.legacy_cmd_topic})'
            )
        self.get_logger().info(f'현재 모드: {"수동 조작" if self.teleop else "자율주행"}')
    
    def joy_callback(self, data):
        """조이스틱 콜백 - PS5 듀얼센스 기준"""
        self.axes = data.axes
        self.button = data.buttons
        
        if not self.button_previous_state:
            self.button_previous_state = list(self.button)
        
        # 수동 모드 버튼
        if len(self.button) > self.manual_button_idx and self.button[self.manual_button_idx] == 1:
            self.teleop = True
            self.bool_pubmsg = False
        
        # 자율주행 모드 버튼
        elif len(self.button) > self.auto_button_idx and self.button[self.auto_button_idx] == 1:
            self.teleop = False
            self.bool_pubmsg = False
        
        # 왼쪽 아날로그 스틱 - 속도 제어
        if len(data.axes) > 1 and data.axes[1] is not None:
            self.move = True
            self.vel = int(data.axes[1] * self.MAX_Velocity)
        
        # 오른쪽 아날로그 스틱 좌우(PS5 기본: axes[3]) - 조향 제어
        if len(data.axes) > self.steering_axis and data.axes[self.steering_axis] is not None:
            self.zero2 = True
            self.steer = float(data.axes[self.steering_axis] * self.MAX_Steering)

        # Triangle 버튼 - 최대속도 증가
        if len(self.button) > 3 and len(self.button_previous_state) > 3 and self.button_previous_state[3] == 0 and self.button[3] == 1:
            if self.MAX_Velocity + 10 < 230:
                self.MAX_Velocity += 10
                self.get_logger().info(f'최대속도 증가: {self.MAX_Velocity}')
            else:
                self.get_logger().warn(f'최대속도 한계: {self.MAX_Velocity}')
        
        # X 버튼 - 최대속도 감소
        if len(self.button) > 0 and len(self.button_previous_state) > 0 and self.button_previous_state[0] == 0 and self.button[0] == 1:
            if self.MAX_Velocity - 10 > 20:
                self.MAX_Velocity -= 10
                self.get_logger().info(f'최대속도 감소: {self.MAX_Velocity}')
            else:
                self.get_logger().warn(f'최소속도 한계: {self.MAX_Velocity}')
        
        self.button_previous_state = list(self.button)
    
    def control_loop(self):
        """메인 제어 루프 - 10Hz"""
        # Mode topic should be visible continuously in ros2 topic echo.
        self.mode_msg.data = self.teleop
        self.mode_publisher.publish(self.mode_msg)

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
            if self.legacy_vel_publisher is not None:
                self.legacy_vel_publisher.publish(vel_msg)
        else:
            # In autonomous mode, do not overwrite planner commands.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
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
