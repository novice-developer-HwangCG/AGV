import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.zeroset_sub = self.create_subscription(Bool,'/zero_set', self.zeroset_callback, 10)
        
        # 퍼블리셔: 측정된 거리 값 퍼블리시 (단위: meter)
        self.lidar_pub = self.create_publisher(Float32, '/lidar_distance', 10)

        # lidar 연결
        self.lidar = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.stop_cmd = bytes([0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0x88, 0xAA])
        self.set_mode_cmd = bytes([0x55, 0x0D, 0x00, 0x00, 0x00, 0x00, 0xF2, 0xAA])
        self.start_cmd = bytes([0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xAA])

        # 측정 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.read_lidar_data)

        # Zero Set 기준값
        self.zero_offset = 0.0
        self.lidar.write(self.stop_cmd)
        time.sleep(0.1)
        self.lidar.write(self.set_mode_cmd)
        time.sleep(0.1)
        self.lidar.write(self.start_cmd)
        time.sleep(0.1)

        # Zero Set 서비스 (추후 필요 시 추가 가능)
        self.get_logger().info('LidarNode initialized. Listening for data...')

    def zeroset_callback(self, msg):
        if msg.data:
            self.zero_offset = self.get_current_distance()  # 또는 마지막 측정값 사용
            self.get_logger().info(f'[ZeroSet] 기준점 갱신: offset = {self.zero_offset:.2f} m')

    @staticmethod
    def crc8(ptr):
        crc = 0x00
        for byte in ptr:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
                crc &= 0xFF
        return crc

    def read_lidar_data(self):
        if self.lidar.read(1) != b'\x55':
            return

        frame = self.lidar.read(7)
        if len(frame) != 7 or frame[-1] != 0xAA:
            return

        key = frame[0]
        value = list(frame[1:6])
        crc = frame[5]

        check_crc = self.crc8([key] + value[:4])

        if check_crc != crc:
            return

        status = value[0]
        distance = int.from_bytes(value[1:4], byteorder='big')  # mm

        if status == 0x00:
            meters = distance / 1000.0
            adjusted = max(0.0, meters - self.zero_offset)
            self.lidar_pub.publish(Float32(data=adjusted))
            self.get_logger().info(f"Distance: {meters:.2f} m (adj: {adjusted:.2f} m)")
        else:
            self.get_logger().warn(f"Sensor error code: {status}")
        
        self.last_distance = meters

    def get_current_distance(self):
        return getattr(self, 'last_distance', 0.0)

    def destroy_node(self):
        self.lidar.write(self.stop_cmd)
        time.sleep(0.1)
        self.lidar.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()