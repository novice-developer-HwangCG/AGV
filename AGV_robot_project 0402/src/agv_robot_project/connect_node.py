import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory
import socket
import socketio
import threading
import os
import subprocess
import time

"""
해야할 일
1. pc에서 입력된 거리(단위는 m) 값을 server로 부터 받아 입력 받은 거리 만큼 라인 트레이싱 수동은 해당 안됨, 사용자가 2의 값을 보냈을 때 2m 값을 저장 'Go' 명령을 받으면면 저장된 거리 값 2m 만큼 라인트레이싱
(입력된 거리 값이 없으면 사용자가 'Stop' 버튼을 누르거나 라인이 없을 때 까지 주행) 'Stop' 명령을 받으면 정지지 (정지 했을 때 입력된 거리 만큼 가지 못했어도 입력된 거리는 초기화)
2. zero set 받아서 반영 (lidar를 이용한 거리 영점 조절 표적 위치 아래 로봇을 이동시킨 뒤 현재 측정되는 로봇의 거리[위치]를 Zero Set[0m]로 설정하고 해당 위치를 기준으로 로봇을 이동 시켜 표적으로부터의 거리를 상대적 확인 함, 로봇의 현재 거리가 10.3m 일 때 'Zero Set' 버튼을 눌러 zero set 거리를 0m로 설정 이후 로봇이 2m 뒤로 후진 하게 되면 로봇의 현재 거리는 8.3m, zero set 거리는 -2.0m)
3. 라이다 거리 반영
4. 이더넷? 노드 반영
5. 모터에 속도 전달 maybe pico 반영
6. 카메라 연결 및 카메라 전달
"""

package_share_directory = get_package_share_directory('agv_robot_project')
dotenv_path = os.path.join(package_share_directory, 'config', '.env')
runstream_path = os.path.join(package_share_directory, 'scripts', 'runstream.sh')

load_dotenv(dotenv_path)

class ConnectNode(Node):
    def __init__(self):
        super().__init__('connect_node')

        # 소켓 설정 (서버와 연결)
        self.SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
        self.ID = os.getenv("ID")
        self.NAME = os.getenv("NAME")

        self.sio = socketio.Client()

        # ROS2 퍼블리셔: drive_node에 명령 전달
        self.command_pub = self.create_publisher(Bool, '/drive_command', 10)
        self.manual_pub = self.create_publisher(Bool, '/manual_command', 10)
        self.move_pub = self.create_publisher(Bool, '/move_command', 10)
        self.watchdog_stop_pub = self.create_publisher(Bool, "/watchdog_stop", 10)  # Watchdog용 비상 정지
        self.zeroset_pub = self.create_publisher(Bool, '/zero_set', 10)
        self.maunal_vel_pub = self.create_publisher(Float32MultiArray, '/manual_vel', 10)

        self.last_pong_time = time.time()
        self.ping_timeout = 3.0  # 3초 동안 서버 응답이 없으면 비상 정지
        self.server_connected = False

        self.manual = False
        self.tracing = False
        self.go_stop = False
        self.zero_set = False

        self.speed = 0.0
        self.steering = 0.0

        self.last_pong_time = time.time()
        self.ping_timeout = 3.0  # 3초 동안 서버 응답이 없으면 비상 정지
        self.server_connected = False

        self.timer_ping = self.create_timer(1.0, self.send_ping)                # 서버와 연결 상태 확인을 위한 ping time
        self.timer_check = self.create_timer(1.0, self.check_server_connection) # 서버와 연결이 끊어졌을 시 확인 하는 check time

        self.register_socketio_handlers()

        self.get_logger().info("SocketIO Started!!!")

    def register_socketio_handlers(self):
        @self.sio.event
        def connect():
            self.get_logger().info(f"[DEBUG] Connecting to: {self.SERVER_SOCKETIO}")
            self.get_logger().info('Socket Server connected')
            self.last_pong_time = time.time()  # 연결 시 마지막 pong 수신 시간 초기화
            self.server_connected = True
            self.update_watchdog_stop(False)
            # 서버에 로봇 등록
            self.sio.emit("register_robot", {"robot_id": self.ID, "robot_name": self.NAME})

        """비정상 종료 / 서버와 연결끊김(비상 정지)"""
        @self.sio.event
        def disconnect():
            self.get_logger().error("Server connection lost! Triggering Emergency Stop!")
            self.server_connected = False  # 서버 연결 상태 업데이트
            self.update_watchdog_stop(True)

        """사용자에 의해 종료"""
        @self.sio.on('disconnect')
        def on_disconnect():
            # 종료 하면서 자동 정지 명령 발행
            self.server_connected = False
            self.get_logger().info("Robot stopped due to user disconnect.")

        @self.sio.on('register_robot')
        def on_message(data):
            self.get_logger().info(f"Message received: {data}")

        @self.sio.on("pong")
        def on_pong():
            self.get_logger().info("<<< Pong received")
            self.last_pong_time = time.time()

        @self.sio.on("camera_request")
        def camera_handler(data):
            type_ = data.get("type")
            if type_ == "start":
                self.get_logger().info("Starting camera stream...")
                subprocess.Popen(["bash", runstream_path])

        @self.sio.on('tracing')
        def on_tracing(data):
            drive_msg = Bool()
            if data['type'] == 'tracing':
                self.tracing = True
            else:
                self.tracing = False
            self.get_logger().info(f"tracing: {self.tracing}")
            drive_msg.data = self.tracing 
            self.command_pub.publish(drive_msg)

        @self.sio.on('manual')
        def on_manual(data):
            manual_msg = Bool()
            if data['type'] == 'manual':
                self.manual = True
            else:
                self.manual = False
            self.get_logger().info(f"manual: {self.manual}")
            manual_msg.data = self.manual 
            self.manual_pub.publish(manual_msg)

        @self.sio.on('move_command')
        def move_command(data):
            move_msg = Bool()
            if data['type'] == 'go':
                self.go_stop = True
            else:
                self.go_stop = False
            self.get_logger().info(f"move: {self.go_stop}")
            move_msg.data = self.go_stop 
            self.move_pub.publish(move_msg)

        @self.sio.on('zero_set')
        def zero_set(data):
            zero_msg = Bool()
            if data['type'] == 'set':
                self.zero_set = True
                self.get_logger().info(f"zero_set: {self.zero_set}")
                zero_msg.data = self.zero_set 
                self.zeroset_pub.publish(zero_msg)

        @self.sio.on("move")
        def move(data):
            type = data["type"]             # data type = speed, steering
            value = data["value"]
            if type == "speed":
                self.speed = round(value, 3)
            else:
                self.steering = round(value, 3)
            
            maunal_vel_msg = Float32MultiArray()
            maunal_vel_msg.data = [self.speed, self.steering]   # 속도 및 조향 값을 메시지 데이터에 추가
            self.maunal_vel_pub.publish(maunal_vel_msg)      # 메시지를 퍼블리시
            # self.get_logger().info(f"Published cmd_vel: {cmd_vel_msg.data}")

    def send_ping(self):
        """서버에 ping 신호를 보냄"""
        if self.sio.connected:
            self.sio.emit("ping")  # 서버에 ping 요청

    def check_server_connection(self):
        """서버 연결 상태 확인: 5초 동안 서버 응답이 없으면 비상 정지"""
        if time.time() - self.last_pong_time > self.ping_timeout:
            if self.server_connected:
                self.get_logger().info("No response from server for 5 seconds. Emergency Stop!")
                self.server_connected = False

    def update_watchdog_stop(self, status):
        """ControllerNode에게 비상 정지 명령 전송"""
        watchdog_stop_msg = Bool()
        watchdog_stop_msg.data = status
        self.watchdog_stop_pub.publish(watchdog_stop_msg)
        
        if status:
            self.get_logger().info("Watchdog Stop Activated! (Server Disconnected)")
        else:
            self.get_logger().info("Watchdog Stop Deactivated! (Server Reconnected)")

    def start(self):
        try:
            self.sio.connect(self.SERVER_SOCKETIO)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
        rclpy.spin(self)

    def stop(self):
        self.sio.disconnect()
        self.get_logger().info(f"Socketio stopped!")

def main(args=None):
    rclpy.init(args=args)
    connect_node = ConnectNode()
    try:
        connect_node.start()
    except KeyboardInterrupt:
        connect_node.stop()
    connect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()