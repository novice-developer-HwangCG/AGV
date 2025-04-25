import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

"""
주의점 

1. 지금은 Minimal Viable Product 단계로 급하게 코딩을 하는 것이라 PID 또는 S-curve외 필요한 부분은 이후에 적용
2. 카메라 연결 코드 필요
3. 해당 노드는 linetracing을 위한 코드로 계산되는 속도 값은 실제 모터에게 속도를 전달하는 pico_node.py로 전송하는 코드 필요

"""

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        # 명령 수신용 서브스크라이버
        self.create_subscription(Bool, '/drive_command', self.command_callback, 10)
        self.create_subscription(Bool, '/move_command', self.move_callback, 10)

        # 주행 제어용 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 주행 관련 상태
        self.tracing = False
        self.go_stop = False
        self.running = False

        self.timer = self.create_timer(0.05, self.drive_logic)  # 20Hz 주기

    def command_callback(self, data_msg: Bool):
        self.tracing = data_msg.data
        self.get_logger().info(f"Received Tracing: {self.tracing}")

    def move_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data
        self.get_logger().info(f"Go_stop: {self.go_stop}")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def drive_logic(self):
        if self.tracing:
            if self.go_stop:
                self.get_logger().info("Tracing Go")
                # 이미지 처리
                frame = self.latest_image
                height, width, _ = frame.shape
                roi = frame[int(height*0.6):, :]  # 하단 40% 영역만 사용

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
                moments = cv2.moments(binary)

                twist = Twist()

                if moments['m00'] > 0:
                    cx = int(moments['m10'] / moments['m00'])
                    error = cx - width // 2

                    twist.linear.x = 0.2
                    twist.angular.z = -float(error) / 100.0  # 간단한 비례 제어
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.send_motor_command_from_twist(twist)
            else:
                self.get_logger().info("Tracing Stop")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

    def send_motor_command_from_twist(self, twist: Twist):
        # 속도 비례 변환 (예시: 선속도 ±0.2 m/s → 모터 속도 ±100)
        linear = twist.linear.x  # 전진/후진 속도
        angular = twist.angular.z  # 회전 속도

        base_speed = int(linear * 500)  # 예: 0.2m/s → 100
        turn_correction = int(angular * 300)  # 예: 0.5rad/s → 150

        speed_left = base_speed - turn_correction
        speed_right = base_speed + turn_correction

        self.pico.send_motor_command(speed_left, speed_right)

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DriveNode interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
