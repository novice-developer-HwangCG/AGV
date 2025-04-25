import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import Float32

class EthernetNode(Node):
    def __init__(self):
        super().__init__('ethernet_node')

        # UDP 소켓 설정 (예: 점검 장비로 전송)
        self.target_ip = '192.168.0.200'  # 점검 장비 IP
        self.target_port = 12345          # 점검 장비 포트
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # LIDAR 거리 구독
        self.create_subscription(Float32, '/lidar_distance', self.lidar_callback, 10)

        self.get_logger().info(f"EthernetNode initialized. Sending to {self.target_ip}:{self.target_port}")

    def lidar_callback(self, msg: Float32):
        distance = msg.data
        message = f"DIST:{distance:.2f}"
        try:
            self.sock.sendto(message.encode('utf-8'), (self.target_ip, self.target_port))
            self.get_logger().info(f"Sent: {message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP message: {e}")

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EthernetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
