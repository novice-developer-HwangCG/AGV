import rclpy
from rclpy.node import Node
import subprocess
import time
from geometry_msgs.msg import Twist

class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog_node')

        """ 감시할 노드 리스트 
            "/ethernet_node",
            "/drive_node",
            "/lidar_node",
            "/pico_node"
        """
        self.monitored_nodes = [
            "/connect_node"
        ]
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel_watchdog", 10)
        self.timer = self.create_timer(1.0, self.check_nodes_status)  # 1초마다 감시

    def check_nodes_status(self):
        """ROS2에서 노드 상태 확인 및 비상 정지 실행"""
        try:
            result = subprocess.run(["ros2", "node", "list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=5.0)
            active_nodes = set(line.strip() for line in result.stdout.strip().split("\n"))  # 빈 줄 제거

            for node in self.monitored_nodes:
                if node not in active_nodes and node.lstrip("/") not in active_nodes:
                    if node == "/pico_node":
                        self.get_logger().error(f"Pico node is down! Cannot send emergency stop.")
                    else:
                        self.get_logger().error(f"{node} is not running! Triggering Emergency Stop.")
                        self.send_emergency_stop()
        except Exception as e:
            self.get_logger().error(f"Error checking node status: {e}")

    def send_emergency_stop(self):
        """비상 정지 실행"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("Emergency Stop Sent due to node failure.")

def main(args=None):
    rclpy.init(args=args)
    watchdog = WatchdogNode()
    try:
        rclpy.spin(watchdog)
    except KeyboardInterrupt:
        watchdog.get_logger().info("Shutting down WatchdogNode...")
    finally:
        if rclpy.ok():  # shutdown 전에만 로그 출력 허용
            watchdog.get_logger().info("Shutting down WatchdogNode...")
        watchdog.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
