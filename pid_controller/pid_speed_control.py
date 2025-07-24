#!/usr/bin/env python3
"""
ROS2 노드: PID 속도 제어 및 Ackermann 조향 컨트롤러

사용법:
  ros2 run pid_controller pid_speed_control \
    --ros-args -p kp:=1.0 -p ki:=0.1 -p kd:=0.01 -p wheelbase:=0.325

구독:
  /cmd_vel               # geometry_msgs/Twist (목표 속도/회전속도)
  /ego_racecar/odom      # nav_msgs/Odometry (실제 속도 피드백)

발행:
  /drive                 # ackermann_msgs/AckermannDriveStamped (제어 출력)

매개변수:
  kp, ki, kd      # PID 계수
  wheelbase       # 차량 휠베이스 (m)
"""
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_speed_controller')
        # 파라미터 선언
        self.kp = self.declare_parameter('kp', 1.0).value
        self.ki = self.declare_parameter('ki', 0.0).value
        self.kd = self.declare_parameter('kd', 0.0).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.325).value

        # 내부 변수 초기화
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0

        # 구독/발행 설정
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_timer(0.02, self.control_loop)  # 50 Hz 제어 주기

    def cmd_vel_callback(self, msg: Twist):
        # 목표 속도/회전속도 설정
        self.target_speed = msg.linear.x
        # steering angle 계산
        if abs(msg.linear.x) > 0.1:
            self.target_steering = math.atan(self.wheelbase * msg.angular.z / msg.linear.x)
        else:
            self.target_steering = 0.0

    def odom_callback(self, msg: Odometry):
        # 실제 속도 계산 (xy 합)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        error = self.target_speed - self.current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID 연산
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = now

        # Ackermann 메시지 생성 및 발행
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(output)
        drive_msg.drive.steering_angle = float(self.target_steering)
        self.pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
