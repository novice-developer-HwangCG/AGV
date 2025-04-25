import rclpy
import serial
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class PicoController(Node):
    def __init__(self):
        super().__init__('pico_node')
        self.cmd_sub = self.create_publisher(Twist, '/cmd_vel', self.cmd_vel_sub_callback,10)
        self.watchdog_cmd_sub = self.create_subscription(Twist, "/cmd_vel_watchdog", self.watchdog_cmd_callback, 10)
        self.manual_sub = self.create_publisher(Bool, '/manual_command', self.manual_sub_callback,10)
        self.watchdog_stop_sub = self.create_subscription(Bool, "/watchdog_stop", self.watchdog_stop_sub_callback, 10)
        self.manual_vel_sub = self.create_publisher(Float32MultiArray, '/manual_vel', self.manual_vel_sub_callback,10)

        self.log_publisher = self.create_publisher(String, "/pico_log", 10)

        self.port = "/dev/ttyUSB1"
        self.baudrate = 115200
        self.ser = self.init_serial()

        self.uart_connected = False

        self.comm_check_interval = 0.1
        self.time_uart_check = self.create_timer(self.comm_check_interval, self.check_uart_connection)  # UART 상태 체크 타이머

        self.manual = False
        self.watchdog_stop = False
        self.emergency_stop_triggered = False

    def init_serial(self):
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,   
                timeout=0.1
            )
            self.publish_log("conncet success to pico.")
            self.uart_connected=True
            return ser
        except serial.SerialException as e:
            self.publish_log(f"Failed to initialize serial connection: {str(e)}")
            self.uart_connected = False
            return None

    # Tracing(자율주행) 할 때 받는 속도 값
    def cmd_vel_sub_callback(self, cmd_vel_msg:Twist):
        if self.emergency_stop_triggered:
            self.publish_log("Ignoring /cmd_vel because emergency stop is active.")
            return  # 비상 정지 상태에서는 새로운 속도 업데이트를 무시

        self.speed = cmd_vel_msg.data[0]
        self.steering = cmd_vel_msg.data[1]

    # Manual(수동조작) 할 때 받는 속도 값
    def manual_vel_sub_callback(self, manual_vel_msg:Float32MultiArray):
        if self.emergency_stop_triggered:
            self.publish_log("Ignoring /manual_vel because emergency stop is active.")
            return  # 비상 정지 상태에서는 새로운 속도 업데이트를 무시
        
        self.speed = manual_vel_msg.data[0]
        self.steering = manual_vel_msg.data[1]

    def manual_sub_callback(self, data_msg: Bool):
        self.manual = data_msg.data
        self.publish_log(f"Received manual: {self.manual}")

    def watchdog_cmd_callback(self):
        """ Watchdog에서 비상 정지 신호 수신(실행 중인 node 중 하나라도 죽을 시)"""
        self.publish_log("Emergency Stop triggered by Watchdog!")
        self.emergency_stop()

    def watchdog_stop_sub_callback(self, msg):
        """socketio_node에서 server와 연결이 끊겼을 시"""
        self.watchdog_stop = msg.data
        self.publish_log(f"Received Watchdog Stop: {self.watchdog_stop}")

        if self.watchdog_stop:
            self.publish_log("Server Disconnected! Executing Emergency Stop!")
            self.emergency_stop()
        else:
            self.publish_log("Server Reconnected! Cancelling Emergency Stop!")
            self.emergency_stop_triggered = False  # 비상 정지 해제

    def emergency_stop(self):
        """비상 정지 실행"""
        self.emergency_stop_triggered = True
        self.send_to_pico([0xFF, 1, 0, 0, 1, 0])  # 즉시 정지 명령
        self.publish_log("Emergency Stop Executed!")

    def check_uart_connection(self):
        # pico와의 uart 통신, 통신이 끊기면 데이터 값을 전달을 못하니 pico 내에서 통신이 끊긴 경우에 대한 코드 추가
        try:
            if self.ser and self.ser.in_waiting >= 0:
                if not self.uart_connected:  # 재연결
                    self.uart_connected = True
                    self.publish_log("UART reconnected.")
            else:
                raise serial.SerialException("UART not responsive.")
        except serial.SerialException:
            if self.uart_connected:  # 연결 실패
                self.uart_connected = False
                self.publish_log("UART disconnected.")

    def send_to_pico(self, data):
        # PWM 최대 값 제한
        if data is None:
            self.publish_log("send_to_pico() received None! Sending stop command instead.")
            data = [0xFF, 0, 0, 0, 1, 0]  # 기본 정지 명령
            
        data[3] = min(data[3], 255)
        data[5] = min(data[5], 255)        

        if not self.uart_connected:
            self.publish_log("Cannot send data to Pico: UART disconnected.")
            return
        try:
            if self.ser:
                self.ser.write(bytes(data))         # 보내는 데이터 배열은 6개 pico가 받는 것은 5개
                # self.publish_log(f"Sent to Pico: {data}")
                # print(f"data: {data}")
                # self.ser.write(bytes([0xFF,("enable"),("L_dir"),("L_pwm"),("R_dir"),("R_pwm")]))
                time.sleep(0.01)
            else:
                self.uart_connected = False
                self.publish_log("UART connection lost.")
                self.ser.close()          # UART 끊기면 닫은 후 복구 시도
        except Exception as e:
            self.publish_log(f"Error sending data to Pico: {str(e)}")
            self.uart_connected = False  # 연결 실패
            self.ser.close()    # 추가 예외 발생 시 UART 연결 끊기

    def read_from_pico(self):
        """Pico에서 데이터 수신 및 로깅"""
        try:
            if self.ser and self.ser.in_waiting > 0:
                # Pico에서 들어오는 데이터 읽기
                data = self.ser.readline().decode('utf-8').strip()
                if data:
                    self.publish_log(f"Received from Pico: {data}")
                    return data
        except Exception as e:
            self.publish_log(f"Error reading from Pico: {str(e)}")
        return None

    """로깅 메세지 토픽"""
    def publish_log(self, message):
        if hasattr(self, 'log_publisher'):  # log_publisher가 초기화되었는지 확인
            log_msg = String()
            log_msg.data = message
            self.log_publisher.publish(log_msg)

    def close(self):
        self.time_uart_check.shutdown()
        self.send_to_pico([0xFF, 1, 0, 0, 1, 0])
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    picocontrol = PicoController()
    try:
        rclpy.spin(picocontrol)
    except KeyboardInterrupt:
        picocontrol.close()
    picocontrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
