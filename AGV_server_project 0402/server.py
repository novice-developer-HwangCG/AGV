from eventlet import monkey_patch
monkey_patch()
import json
from flask import Flask
from flask_socketio import SocketIO
from redis_db import get_redis_connection
from flask import request
from static import STATIC_VAR
import os
import common

app = Flask(__name__)

REDIS_HOST = os.environ.get("REDIS_HOST", "localhost")
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD", "agvtestserverproject0402")
RTMP = os.environ.get("SERVER_RTMP", "")

# Redis 연결
redis = get_redis_connection()

socketio = SocketIO(app, cors_allowed_origins="*")

# 연결
@socketio.on("connect")
def client_connect(message):
    print("Client connect : ", request.sid, flush=True)

# 연결 종료
@socketio.on("disconnect")
def client_disconnect():
    print("Client disconnect : ", request.sid, flush=True)

# server에 robot 등록
@socketio.on("register_robot")
def handle_register_robot(data):
    robot_id = data.get("robot_id")
    robot_name = data.get("robot_name")
    client_id = request.sid  # 연결된 Socket의 고유 ID

    print(f"[ROBOT REGISTERED] ID: {robot_id}, Name: {robot_name}, SID: {client_id}")

    robots = redis.get("robots")
    data_robots = json.loads(robots) if robots else []

    # 기존 robot_id가 있으면 업데이트, 없으면 새로 추가
    updated = False
    for robot in data_robots:
        if robot["id"] == robot_id:
            robot["name"] = robot_name
            robot["stream"] = f"{RTMP}/live/{robot_id}"
            robot["robot_socket_id"] = client_id
            updated = True
            break

    if not updated:
        data_robots.append({
            "id": robot_id,
            "robot_socket_id": client_id,
            "name": robot_name,
            "stream": f"{RTMP}/live/{robot_id}"
        })

    redis.set("robots", json.dumps(data_robots))
    socketio.emit("register_robot", {"status": 1}, room=client_id)

# """클라이언트(pc)가 요청한 로봇 연결 요청을 받아서 서버에 연결된 로봇을 반환(한 대만)"""
# @socketio.on("get_robot")
# def get_robot():
#     client_id = request.sid
#     robots = redis.get("robots")  # Redis에서 로봇 목록을 가져옴
#     if robots is None:
#         robots = []
#     else:
#         robots = json.loads(robots)
#     socketio.emit("get_robot", {"robots": robots}, room=client_id)
#     print(f"[Robot Request] Client {request.sid} requested connected robot info.")

@socketio.on("get_robot_list")
def handle_get_robot_list():
    client_id = request.sid
    robots_data = redis.get("robots")
    robots = json.loads(robots_data) if robots_data else []

    # 필요한 경우: 사용 가능(미연결) 로봇만 필터링
    # robots = [r for r in robots if r["client_id"] in (None, "", "null")]

    socketio.emit("get_robot_list", {"robots": robots}, room=client_id)
    print(f"[SERVER] Robot list sent to client {client_id}")

@socketio.on("select_robot")
def handle_select_robot(data):
    robot_id = data.get("robot_id")
    client_id = request.sid

    robots_data = redis.get("robots")
    if not robots_data:
        socketio.emit("select_robot", {"status": 0, "error": "No robots registered"}, room=client_id)
        return

    robots = json.loads(robots_data)
    success = False

    for robot in robots:
        if robot["id"] == robot_id:
            current_owner = str(robot.get("client_id"))
            if not current_owner or current_owner not in active_clients():
                robot["client_id"] = client_id
                success = True
            else:
                socketio.emit("select_robot", {
                    "status": 0,
                    "error": "Robot already in use"
                }, room=client_id)
                return

    if success:
        redis.set("robots", json.dumps(robots))
        socketio.emit("select_robot", {"status": 1, "robot_id": robot_id}, room=client_id)
        print(f"[SERVER] Robot {robot_id} assigned to client {client_id}")
    else:
        socketio.emit("select_robot", {"status": 0, "error": "Robot not found"}, room=client_id)

@socketio.on("release_robot")
def handle_release_robot(data):
    robot_id = data.get("robot_id")
    client_id = request.sid

    print(f"[SERVER] Client {client_id} requests to release robot {robot_id}", flush=True)

    robots_data = redis.get("robots")
    if robots_data:
        robots = json.loads(robots_data)
        updated = False

        for robot in robots:
            if robot["id"] == robot_id and str(robot.get("client_id")) == str(client_id):
                robot["client_id"] = None  # 또는 "", 상태만 초기화
                updated = True

        if updated:
            redis.set("robots", json.dumps(robots))
            print(f"[SERVER] Robot {robot_id} released by client {client_id}", flush=True)
        else:
            print(f"[SERVER] Robot {robot_id} not found or not owned by this client.", flush=True)
    else:
        print("[SERVER] No robots found in Redis.", flush=True)

    socketio.emit("release_robot", {"status": 1}, room=client_id)

# 연결 상태 확인 서버가 pong을 응답 connect_node로 보냄냄
@socketio.on("ping")
def handle_ping():
    print(">>> Received ping")
    socketio.emit("pong")

@socketio.on("camera_request")
def stream():
    robot_id = data["robot_id"]
    client_id = request.sid
    robot_sid = get_robot_socket_id_by_robot_id(robot_id)
    socketio.emit("camera_request", {"type": type_}, room=robot_sid)
    socketio.emit("camera_request", {"status": 1}, room=client_id)

# 라인트레이싱 모드 on off
@socketio.on("tracing")
def run_tracing(data):
    robot_id = data["robot_id"]
    type_ = data["type"]
    client_id = request.sid
    print(f"[SERVER] Received Tracing mode request: {data} from client {client_id}")
    
    robot_sid = get_robot_socket_id_by_robot_id(robot_id)
    if not robot_sid:
        print(f"[SERVER] Tracing command failed: No robot socket ID for {robot_id}")
        return
    print(f"[SERVER] → Target robot socket id: {robot_sid}")
    socketio.emit("tracing", {"type": type_}, room=robot_sid)
    socketio.emit("tracing", {"status": 1}, room=client_id)

# 수동 조작 모드 on off
@socketio.on("manual")
def maunal_mode(data):
    robot_id = data["robot_id"]
    type_ = data["type"]
    client_id = request.sid
    print(f"[SERVER] Received Manual mode request: {data} from client {client_id}")

    robot_sid = get_robot_socket_id_by_robot_id(robot_id)
    if not robot_sid:
        print(f"[SERVER] Manual command failed: No robot socket ID for {robot_id}")
        return
    print(f"[SERVER] → Target robot socket id: {robot_sid}")
    socketio.emit("manual", {"type" : type_}, room=robot_sid)
    socketio.emit("manual", {"status" : 1}, room=client_id)

# 라인 트레이싱 go stop
@socketio.on("move_command")
def move_command(data):
    robot_id = data["robot_id"]
    type_ = data["type"]
    client_id = request.sid
    robot_sid = get_robot_socket_id_by_robot_id(robot_id)
    socketio.emit("move_command", {"type" : type_}, room=robot_sid)        # 주행 명령 (go, stop)

# 수동 조작 모드 키 입력 속도 값
@socketio.on("movement_type")
def movement_type(data):
    try:
        robot_id = data["robot_id"]
        client_id = request.sid

        robot_sid = get_robot_socket_id_by_robot_id(robot_id)
        status = 0  # 명령 처리 상태 초기화 (0: 실패)

        if robot_sid:
            socketio.emit("move", data, room=robot_sid) # 로봇 클라이언트에 "move" 이벤트 전송
            status = 1  # 명령이 성공적으로 전송되었음을 표시
        else:
            print(f"Error: No client associated with robot_id={robot_sid}", flush=True)

        socketio.emit("movement_type", {"status": status}, room=client_id)   # 명령 처리 상태를 원래 클라이언트에게 반환
    except KeyError as e:           # 필수 키가 누락된 경우 처리
        print(f"KeyError: Missing key {e} in data: {data}", flush=True)
        socketio.emit("movement_type", {"status": 0, "error": f"Missing key: {e}"}, room=request.sid)
    except Exception as e:          # 예기치 않은 오류 처리
        print(f"Unexpected error in movement_type: {e}", flush=True)
        socketio.emit("movement_type", {"status": 0, "error": str(e)}, room=request.sid)

# 영점 조절
@socketio.on("zero_set")
def zero_set(data):
    robot_id = data["robot_id"]
    type_ = data["type"]
    client_id = request.sid
    robot_sid = get_robot_socket_id_by_robot_id(robot_id)
    socketio.emit("zero_set", {"type" : type_}, room=robot_sid)        # 영점 조절 명령

# PC GUI가 선택한 로봇에 대해 현재 연결된 클라이언트의 socket id (PC GUI) 를 리턴함.
def get_client_id_by_robot_id(robot_id):
    robots_data = redis.get("robots")
    if robots_data:
        robots = json.loads(robots_data)
        for r in robots:
            if r["id"] == robot_id:
                print(f"[DEBUG] Found client_id for robot {robot_id}: {r.get('client_id')}", flush=True)
                return r.get("client_id")
    print(f"[DEBUG] No client_id found for robot {robot_id}", flush=True)
    return None

# 실제 로봇 측에서 서버에 연결한 socket id (ROS 노드) 를 리턴함
def get_robot_socket_id_by_robot_id(robot_id):
    robots_data = redis.get("robots")
    if robots_data:
        robots = json.loads(robots_data)
        for r in robots:
            if r["id"] == robot_id:
                return r.get("robot_socket_id")
    return None

def active_clients():
    """현재 연결된 모든 클라이언트 socket ID 리스트"""
    return set(socketio.server.manager.rooms["/"].keys())
    # return socketio.server.manager.rooms["/"]

def reset_robot_clients():
    robots_data = redis.get("robots")
    if robots_data:
        robots = json.loads(robots_data)
        for robot in robots:
            robot["client_id"] = None
        redis.set("robots", json.dumps(robots))
        print("[SERVER] All robot client_id reset")

if __name__ == '__main__':
    import eventlet
    import eventlet.wsgi

    try:
        redis = get_redis_connection()
    except Exception as e:
        print(f"[Redis Connect Failed!] {e}")
        redis = None  # 또는 서버 강제 종료
    print(f"Redis 연결 대상: {REDIS_HOST}")
    print(f"RTMP 주소: {RTMP}")
    print("SERVER ON")
    reset_robot_clients()
    socketio.run(app, host="0.0.0.0", port=5000)
