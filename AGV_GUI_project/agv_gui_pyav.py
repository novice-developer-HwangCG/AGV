from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QComboBox, QTextEdit, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap
import sys
import av
import time
from socketio_client import SocketMethod

"""
해야할 일 
1. agv_project ui 크기 줄이기 ( 보류 )
2. 'CAD Distance' log 박스 = 현재 거리(로봇이 이동한 거리), 표적 과의 거리 함께 출력, 'Zero Distance' log 박스 = 영점 조절 거리, 'Robot log' 연결된 로봇이 보내는 메세지, 'PC / SERVER log' pc와 server 로그 메세지 각각 log 메세지 연결 ( 보류 )
3. connect robot / disconnect robot 변경 -> 'Connect Robot' 버튼을 눌러 로봇이 연결이 되면 'Disconnect Robot' 문구 변경 및 해당 버튼을 다시 누르면 로봇 연결 끊고 'Connect Robot'문구로 다시 전환 = ( 완료 )
4. tracing 모드일 때 manual 버튼이 눌리지 않도록 하기 manual 모드 일 때도 동일하게 tracing 버튼 안 눌리도록 하기 = ( 완료 )
5. 거리(단위는 m) 입력 전달 만약 사용자가 2 입력 후 Enter 키 입력시 로봇에게 2의 값 전달 사용자가 'Go' 버튼을 누르면 2m 만큼 주행(라인트레이싱 만 해당 또한 입력된 거리 값이 없으면 사용자가 'Stop' 버튼을 누르거나 라인이 없을 때 까지 주행)
'Stop' 버튼을 누르면 정지 (정지 했을 때 입력된 거리 만큼 가지 못했어도 입력된 거리는 초기화), 
6. zero set 전달 (lidar를 이용한 거리 영점 조절 표적 위치 아래 로봇을 이동시킨 뒤 현재 측정되는 로봇의 거리[위치]를 Zero Set[0m]로 설정하고 해당 위치를 기준으로 로봇을 이동 시켜 표적으로부터의 거리를 상대적 확인 함, 로봇의 현재 거리가 10.3m 일 때 'Zero Set' 버튼을 눌러 zero set 거리를 0m로 설정 이후 로봇이 2m 뒤로 후진 하게 되면 로봇의 현재 거리는 8.3m, zero set 거리는 -2.0m)
7. 'Camera Request' 버튼을 누를 시 카메라 연결, 카메라가 연결되면 'Camera Close' 문구로 변경 및 해당 버튼을 다시 누르면 연결된 카메라 끊고 'Camera Request' 문구로 다시 변환
8. w, a, s, d, x 는 마우스로 버튼을 눌러도 되고 사용자가 노트북으로 키 입력 해도 됨 ( 보류 )
"""

class AGVGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV Project")
        self.setFixedSize(1280, 720)
        self.setStyleSheet("background-color: #2e2e2e; color: white;")
        self.socket = SocketMethod()
        self.socket.gui = self
        self.robot_connected = False
        self.setup_ui()
        self.socket.run()
        self.socket.get_robot_list()

        self.rtmp_url = ""

        self.camera_thread = None
        self.camera_running = False

    def setup_ui(self):
        main_layout = QHBoxLayout()

        # === Left Side ===
        left_layout = QVBoxLayout()

        title_frame = QFrame()
        title_frame.setFrameShape(QFrame.Box)
        title_layout = QVBoxLayout()
        title_label = QLabel("AGV Project")
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_layout.addWidget(title_label)
        title_frame.setLayout(title_layout)

        self.camera_label = QLabel("Camera")
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setStyleSheet("background-color: #444444; border: 2px solid white;")
        self.camera_label.setAlignment(Qt.AlignCenter)

        button_layout = QHBoxLayout()
        self.robot_selector = QComboBox()
        self.robot_selector.setFixedWidth(200)
        self.robot_selector.setStyleSheet("background-color: white; color: black;")
        self.connect_btn = QPushButton("Connect Robot")
        self.camera_btn = QPushButton("Camera Request")
        for btn in [self.connect_btn, self.camera_btn]:
            btn.setFixedHeight(50)
            btn.setStyleSheet("background-color: #555555; color: white;")
        button_layout.addWidget(self.robot_selector)
        button_layout.addWidget(self.connect_btn)
        button_layout.addWidget(self.camera_btn)

        self.connect_btn.clicked.connect(self.handle_connect_button)
        self.camera_btn.clicked.connect(self.toggle_camera)

        left_layout.addWidget(title_frame)
        left_layout.addWidget(self.camera_label)
        left_layout.addLayout(button_layout)

        # === Right Side ===
        right_layout = QVBoxLayout()

        # Mode selection
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Select Mode")
        mode_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.tracing_btn = QPushButton("Tracing")
        self.manual_btn = QPushButton("Manual")
        for btn in [self.tracing_btn, self.manual_btn]:
            btn.setStyleSheet("background-color: #555555; color: white;")
            btn.setFixedWidth(200)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.tracing_btn)
        mode_layout.addWidget(self.manual_btn)

        # self.tracing_btn.clicked.connect(self.socket.run_tracing)
        # self.manual_btn.clicked.connect(self.socket.run_manual)

        self.tracing_btn.setCheckable(True)
        self.manual_btn.setCheckable(True)
        self.tracing_btn.clicked.connect(self.toggle_tracing_mode)
        self.manual_btn.clicked.connect(self.toggle_manual_mode)

        # Command, Distance, Zero Set
        command_layout = QVBoxLayout()

        row1 = QHBoxLayout()
        command_label = QLabel("Command")
        command_label.setFont(QFont("Arial", 12, QFont.Bold))
        row1.addWidget(command_label)
        self.go_btn = QPushButton("Go")
        self.stop_btn = QPushButton("Stop")
        for btn in [self.go_btn, self.stop_btn]:
            btn.setStyleSheet("background-color: #555555; color: white;")
            btn.setFixedWidth(120)
        row1.addWidget(self.go_btn)
        row1.addWidget(self.stop_btn)

        self.go_btn.clicked.connect(lambda: self.socket.move_command("go"))
        self.stop_btn.clicked.connect(lambda: self.socket.move_command("Stop"))

        row2 = QHBoxLayout()
        distance_label = QLabel("Enter Distance (m)")
        distance_label.setFont(QFont("Arial", 12, QFont.Bold))
        row2.addWidget(distance_label)
        self.distance_input = QLineEdit()
        self.distance_input.setFixedWidth(150)
        self.enter_btn = QPushButton("Enter")
        self.enter_btn.setStyleSheet("background-color: #555555; color: white;")
        row2.addWidget(self.distance_input)
        row2.addWidget(self.enter_btn)

        row3 = QHBoxLayout()
        self.zero_btn = QPushButton("Zero Set")
        self.zero_btn.setStyleSheet("background-color: #555555; color: white;")
        row3.addWidget(self.zero_btn)

        self.zero_btn.clicked.connect(self.socket.zero_set)

        command_layout.addLayout(row1)
        command_layout.addLayout(row2)
        command_layout.addLayout(row3)

        # Keypad
        key_box = QGroupBox()
        key_box.setTitle("")
        key_box.setStyleSheet("color: white; border: 1px solid white;")
        key_grid = QGridLayout()
        keys = {
            (0, 1): "w", (1, 0): "a", (1, 1): "s", (1, 2): "d", (2, 1): "x"
        }
        for (row, col), text in keys.items():
            btn = QPushButton(text)
            btn.setFixedSize(55, 55)
            btn.setStyleSheet("background-color: #555555; color: white;")
            btn.clicked.connect(lambda _, t=text: self.socket.movement_type(t))
            key_grid.addWidget(btn, row, col)
        key_box.setLayout(key_grid)

        mid_layout = QHBoxLayout()
        mid_layout.addLayout(command_layout)
        mid_layout.addWidget(key_box)

        # Log Boxes
        log_layout = QGridLayout()
        self.log1 = QTextEdit("CAD Distance")   # Current and Destination
        self.log2 = QTextEdit("Zero Distance")
        self.log3a = QTextEdit("Robot Log")
        self.log3b = QTextEdit("PC / SERVER Log")
        for log in [self.log1, self.log2, self.log3a, self.log3b]:
            log.setStyleSheet("background-color: #1e1e1e; color: white; border: 1px solid white;")
            log.setReadOnly(True)
        log_layout.addWidget(self.log1, 0, 0)
        log_layout.addWidget(self.log2, 0, 1)
        log_layout.addWidget(self.log3a, 0, 2)
        log_layout.addWidget(self.log3b, 1, 2)

        # Combine all right side
        right_layout.addLayout(mode_layout)
        right_layout.addLayout(mid_layout)
        right_layout.addLayout(log_layout)

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    def handle_robot_list(self, data):
        robots = data.get("robots", [])
        self.robot_selector.clear()
        for robot in robots:
            robot_id = robot["id"]
            name = robot.get("name", robot_id)
            self.robot_selector.addItem(f"{name} ({robot_id})", robot_id)

    def handle_connect_button(self):
        if not self.robot_connected:
            if not self.socket.connected:
                self.socket.run()  # 서버 연결 (이미 연결되어 있으면 무시)
            selected_robot_id = self.robot_selector.currentData()
            if selected_robot_id:
                self.socket.select_robot(selected_robot_id)
            else:
                print("No robot selected.")
        else:
            self.socket.release_robot()
            self.robot_connected = False
            self.connect_btn.setText("Connect Robot")
            print("Robot release requested.")

    def handle_robot_connected(self, robot_id):
        self.robot_connected = True
        self.connect_btn.setText("Disconnect Robot")
        # print("Robot connected: " + robot_id)

    def toggle_camera(self):
        if not self.camera_running:
            # RTMP 주소는 선택된 로봇 기준 stream 필드로 설정
            self.rtmp_url = f"rtmp://192.168.0.80:1935/live/{self.socket.robot_id}"
            self.camera_label.clear()
            self.camera_thread = CameraThread(self.rtmp_url)
            self.camera_thread.change_pixmap_signal.connect(self.update_camera_label, Qt.QueuedConnection)
            self.camera_thread.start()
            self.camera_btn.setText("Camera Close")
            self.camera_running = True
        else:
            if self.camera_thread:
                self.camera_thread.stop()
            self.camera_label.clear()
            self.camera_label.setText("Camera")
            self.camera_btn.setText("Camera Request")
            self.camera_running = False

    def update_camera_label(self, pixmap):
        self.camera_label.setPixmap(pixmap.scaled(640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def toggle_tracing_mode(self):
        if self.robot_connected:
            if self.tracing_btn.isChecked():
                self.socket.run_tracing(True)
                self.manual_btn.setEnabled(False)
            else:
                self.socket.run_tracing(False)
                self.manual_btn.setEnabled(True)

    def toggle_manual_mode(self):
        if self.robot_connected:
            if self.manual_btn.isChecked():
                self.socket.run_manual(True)
                self.tracing_btn.setEnabled(False)
            else:
                self.socket.run_manual(False)
                self.tracing_btn.setEnabled(True)

    def closeEvent(self, event):
        if self.camera_thread:
            self.camera_thread.stop()
        self.socket.release_robot()  # 로봇 사용 중이면 서버에 반환 요청
        self.socket.disconnect()
        event.accept()

class CameraThread(QThread):
    change_pixmap_signal = pyqtSignal(QPixmap)

    def __init__(self, rtmp_url):
        super().__init__()
        self.rtmp_url = rtmp_url
        self._run_flag = True

    def run(self):
        while self._run_flag:
            try:
                print(f"[INFO] Trying to open RTMP stream: {self.rtmp_url}")
                container = av.open(self.rtmp_url, timeout=3.0)
                stream = container.streams.video[0]

                for packet in container.demux(stream):
                    if not self._run_flag:
                        break
                    for frame in packet.decode():
                        if not self._run_flag:
                            break
                        print(f"[FRAME] PTS: {frame.pts}, Size: {frame.width}x{frame.height}")
                        img = frame.to_ndarray(format='rgb24')
                        h, w, ch = img.shape
                        bytes_per_line = ch * w
                        qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
                        pixmap = QPixmap.fromImage(qt_img)
                        if not pixmap.isNull():
                            self.change_pixmap_signal.emit(pixmap)
                        else:
                            print("[WARNING] Null pixmap skipped.")
                        time.sleep(1 / 30)  # 제한 FPS

            except av.AVError as e:
                print(f"[ERROR] AVError during stream: {e}")
                print("[INFO] Attempting to reconnect in 3 seconds...")
                time.sleep(3)  # 재연결 딜레이

            except Exception as e:
                print(f"[ERROR] Unexpected error: {e}")
                print("[INFO] Attempting to reconnect in 3 seconds...")
                time.sleep(3)

            finally:
                try:
                    container.close()
                except:
                    pass

    def stop(self):
        self._run_flag = False
        self.wait()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AGVGui()
    window.show()
    sys.exit(app.exec_())
