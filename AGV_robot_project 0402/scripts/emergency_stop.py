import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.stop_pub = self.create_publisher(String, 'drive_command', 10)
        self.get_logger().info("Press 's' to send emergency stop signal.")
        self.listen_for_stop()

    def listen_for_stop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                ch = sys.stdin.read(1)
                if ch == 's':
                    self.send_stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def send_stop(self):
        msg = String()
        msg.data = 'stop'
        self.stop_pub.publish(msg)
        self.get_logger().warn("[EMERGENCY] Stop signal sent!")


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()