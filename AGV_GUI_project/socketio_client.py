import socketio

sio = socketio.Client()

# 서버 연결
@sio.event
def connect():
    print("[CLIENT] Connected to server")

@sio.event
def disconnect():
    print("[CLIENT] Disconnected from server")

# ================
# 실행 클래스
# ================
class SocketMethod:
    def __init__(self):
        self.robot_id = "AGV"
        self.gui = None
        self.tracing_mode = False
        self.manual_mode = False
        self.go_stop = False
        self.manual_data = {"type": "speed", "value": 0.0}
        self.distance = {"distance": 0.0}
        self.gui = None

    def run(self):
        try:
            sio.connect('http://127.0.0.1:5000')
            # print("[CLIENT] Trying to connect to server...")
        except Exception as e:
            print(f"[CLIENT] Connection failed: {e}")

    # 로봇 연결 끊기 이벤트도 추가 해야 함
    @sio.on("get_robot")
    def on_get_robot(data):
        robots = data.get("robots", [])
        if robots:
            if self.gui is not None:
                self.gui.handle_robot_connected(data)
            print(f"[CLIENT] Robot connected: {robots[0]['id']}")  # 한 대만 사용하는 구조 가정
        else:
            print("[CLIENT] No robot connected.")

    def disconnect(self):
        sio.disconnect()

    def get_robot(self):
        print("[CLIENT] Requesting robot info")
        sio.emit("get_robot")

    def camera_request(self):
        print("[CLIENT] Requesting camera...")
        sio.emit("camera_request")

    def run_tracing(self):import socketio

sio = socketio.Client()

# 서버 연결
# @sio.event
# def connect():
#     print("[CLIENT] Connected to server")

@sio.event
def disconnect():
    print("[CLIENT] Disconnected from server")

# ================
# 실행 클래스
# ================
class SocketMethod:
    def __init__(self):
        self.tracing_mode = False
        self.manual_mode = False
        self.go_stop = False
        self.manual_data = {"type": "speed", "value": 0.0}
        self.distance = {"distance": 0.0}
        self.gui = None
        self.connected = False
        sio.on("get_robot_list", self.on_get_robot_list)
        sio.on("select_robot", self.on_select_robot)
        sio.on("camera_request", self.on_camera_response)

    # 서버 연결
    def run(self):
        if not self.connected:
            try:
                sio.connect('http://127.0.0.1:5000')
                self.connected = True
                print("[CLIENT] Connection established.")
            except Exception as e:
                print(f"[CLIENT] Connection failed: {e}")

    def disconnect(self):
        if self.connected:
            sio.disconnect()
            self.connected = False

    # 로봇 연결 (클라이언트 → 서버)
    def get_robot_list(self):
        if self.connected:
            sio.emit("get_robot_list")

    # 로봇 연결 응답 받기 서버 → 클라이언트
    def on_get_robot_list(self, data):
        if self.gui:
            self.gui.handle_robot_list(data)

    def select_robot(self, robot_id):
        if self.connected:
            self.robot_id = robot_id
            print(f"[CLIENT] Requesting to connect to robot: {robot_id}")
            sio.emit("select_robot", {"robot_id": robot_id})

    def on_select_robot(self, data):
        if data.get("status") == 1:
            robot_id = data.get("robot_id")
            self.robot_id = robot_id
            self.tracing_mode = False
            self.manual_mode = False
            print(f"[CLIENT] Successfully connected to robot: {robot_id}")
            if self.gui:
                self.gui.handle_robot_connected(robot_id)
        else:
            error = data.get("error", "Unknown error")
            print(f"[CLIENT] Failed to connect to robot: {error}")

    def release_robot(self):
        if self.connected:
            data = {"robot_id": self.robot_id}
            print("[CLIENT] Requesting to release robot...")
            sio.emit("release_robot", data)
        else:
            print("[CLIENT] Cannot release robot: Not connected to server")

    def camera_request(self):
        print("[CLIENT] Requesting camera...")
        data = {"robot_id": self.robot_id}
        sio.emit("camera_request", data)

    def on_camera_response(self, data):
        if self.gui:
            self.gui.handle_camera_response(data)

    def run_tracing(self, enable: bool):
        self.tracing_mode = enable
        robot_id = getattr(self, "robot_id", None)
        if not robot_id:
            print("[CLIENT] No robot selected for tracing mode.")
            return
        data = {
            "robot_id": robot_id,
            "type": "tracing" if enable else "tracing off"
        }
        print(f"[CLIENT] Tracing Mode: {data['type']}")
        sio.emit("tracing", data)

    def run_manual(self, enable: bool):
        self.manual_mode = enable
        robot_id = getattr(self, "robot_id", None)
        if not robot_id:
            print("[CLIENT] No robot selected for manual mode.")
            return
        data = {
            "robot_id": robot_id,
            "type": "manual" if enable else "manual off"
        }
        print(f"[CLIENT] Manual Mode: {data['type']}")
        sio.emit("manual", data)

    def move_command(self, cmd_type):
        data = {
            "robot_id": self.robot_id,
            "type": cmd_type
        }
        print(f"[CLIENT] Move Command: {cmd_type}")
        sio.emit("move_command", data)

    def movement_type(self, direction):
        mapping = {
            "w": ("speed", 1.0),
            "x": ("speed", -1.0),
            "a": ("steering", 1.0),
            "d": ("steering", -1.0),
            "s": ("speed", 0.0)
        }
        if direction in mapping:
            self.manual_data = {
                "robot_id": self.robot_id,
                "type": mapping[direction][0],
                "value": mapping[direction][1]
            }
            print(f"[CLIENT] Sending manual input: {self.manual_data}")
            sio.emit("movement_type", self.manual_data)

    def send_distance(self):
        sio.emit("distance", self.distance)

    def zero_set(self):
        data = {
            "robot_id": self.robot_id,
            "type": "set"
        }
        print("[CLIENT] Zero set issued.")
        sio.emit("zero_set", data)

if __name__ == "__main__":
    sm = SocketMethod()
    sm.run()

"""
zero_set 명령 예시
표적 위치 아래에 AGV 이동 시킨 뒤 현재 측정되는 AGV의 거리(위치)를 Zero Set(0m)로 설정하고 해당 위치를 기준으로 AGV를 이동 시켜 표적으로부터의 거리를 상대적으로 확인함
예시)
- 기존
표적
 │
 │
AGV (현재 거리 10.3m, Zero Set 0.0m)

- 후진
표적
 │
 │
    AGV (현재 거리 8.3m, Zero Set -2.0m)
"""


""" 

통신 필요 목록

1. 로봇 연결 Connect / Disconnect
-> Server = O, PC = X, roobt = O

2. 카메라 요청 Camera Request
-> Server = X, PC = X, roobt = X

3. Tracing On / Off
-> Server = O, PC = X, roobt = O

4. Manual On / Off
-> Server = O, PC = X, roobt = O

5. Go / Stop
-> Server = O, PC = X, roobt = O

6. Manual 모드 수동 조작 값
-> Server = O, PC = X, robot = X

7. 엔코더 값? 거리 입력
-> Server = X, PC = X, roobt = X

8. Zeroset
-> Server = O, PC = X, roobt = O
표적 위치 아래에 AGV 이동 시킨 뒤 현재 측정되는 AGV의 거리(위치)를 Zero Set(0m)로 설정하고 해당 위치를 기준으로 AGV를 이동 시켜 표적으로부터의 거리를 상대적으로 확인함
예시)
- 기존
표적
 │
 │
AGV (현재 거리 10.3m, Zero Set 0.0m)

- 후진
표적
 │
 │
    AGV (현재 거리 8.3m, Zero Set -2.0m)

"""

        self.tracing_mode = not self.tracing_mode
        data = {
            "robot_id": self.robot_id,
            "type": "tracing" if self.tracing_mode else "tracing off"
        }
        print(f"[CLIENT] Tracing Mode: {data['type']}")
        sio.emit("tracing", data)

    def run_manual(self):
        self.manual_mode = not self.manual_mode
        data = {
            "robot_id": self.robot_id,
            "type": "manual" if self.manual_mode else "maunal off"
        }
        print(f"[CLIENT] Manual Mode: {data['type']}")
        sio.emit("manual", data)

    def move_command(self, cmd_type):
        data = {
            "robot_id": self.robot_id,
            "type": cmd_type
        }
        print(f"[CLIENT] Move Command: {cmd_type}")
        sio.emit("move_command", data)

    def movement_type(self, direction):
        mapping = {
            "w": ("speed", 1.0),
            "x": ("speed", -1.0),
            "a": ("steering", 1.0),
            "d": ("steering", -1.0),
            "s": ("speed", 0.0)
        }
        if direction in mapping:
            self.manual_data = {
                "robot_id": self.robot_id,
                "type": mapping[direction][0],
                "value": mapping[direction][1]
            }
            print(f"[CLIENT] Sending manual input: {self.manual_data}")
            sio.emit("movement_type", self.manual_data)

    def send_distance(self):
        sio.emit("distance", self.distance)

    def zero_set(self):
        data = {
            "robot_id": self.robot_id,
            "type": "set"
        }
        print("[CLIENT] Zero set issued.")
        sio.emit("zero_set", data)

if __name__ == "__main__":
    sm = SocketMethod()
    sm.run()

"""
zero_set 명령 예시
표적 위치 아래에 AGV 이동 시킨 뒤 현재 측정되는 AGV의 거리(위치)를 Zero Set(0m)로 설정하고 해당 위치를 기준으로 AGV를 이동 시켜 표적으로부터의 거리를 상대적으로 확인함
예시)
- 기존
표적
 │
 │
AGV (현재 거리 10.3m, Zero Set 0.0m)

- 후진
표적
 │
 │
    AGV (현재 거리 8.3m, Zero Set -2.0m)
"""


""" 
현재 구현 목록

1. 로봇 연결 Connect / Disconnect
-> Server = O, PC = X, roobt = O

2. 카메라 요청 Camera Request
-> Server = X, PC = X, roobt = X

3. Tracing On / Off
-> Server = O, PC = X, roobt = O

4. Manual On / Off
-> Server = O, PC = X, roobt = O

5. Go / Stop
-> Server = O, PC = X, roobt = O

6. Manual 모드 수동 조작 값
-> Server = O, PC = X, robot = X

7. 엔코더 값? 거리 입력
-> Server = X, PC = X, roobt = X

8. Zeroset
-> Server = O, PC = X, roobt = O
"""
