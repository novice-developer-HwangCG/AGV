Work in progress — detailed description coming soon.

총 구상도

PC ↔ SERVER ↔ Robot

ros2 launch agv_robot_project agv_launch.py


PC (GUI)  →  SERVER (PC↔Robot 연결)  →  connect_node.py (AGV ROS2 패키지 내부)

connect_node.py  
→  drive_node.py (라인트레이싱)
→  lidar_node.py (LIDAR 거리 측정)
→  ethernet_node.py (외부 점검 장비)
→  watchdog_node.py (긴급 정지 감시)


의문점

노트북 - 젯슨 자비에 nx = 유선 랜선사용?????

motor - pico - jetson

agv_project src 내부에 pico 통신 추가 (모터 제어 속도 값 전송)


통신 이벤트

server              robot
connect             connect(sio.이벤트)
disconnect          disconnect(sio.이벤트 비정상 종료, disconnect 사용자 종료)
register_robot      connect(이벤트 함수 내 존재)
ping                pong
tracing             tracing
manual              manual
move_command        move_command
movement_type       move
zero_set            zero_set

connect - 서버 연결
disconnect - 비정상 종료, 사용자 종료
register_robot - 로봇 등록
ping - 연결 상태 체크
tracing - 라인트레이싱 모드 on off
manual - 수동 조작 모드 on off
move_command - 라인트레이싱 Go Stop
movement_type - 수동 조작 키 입력 속도
zero_set - 영점 조절

    1. connect node 오류
[connect_node-1] Traceback (most recent call last):
[connect_node-1]   File "/home/txgt/ros2_humble/install/agv_test/lib/agv_test/connect_node", line 6, in <module>
[connect_node-1]     from pkg_resources import load_entry_point
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 3254, in <module>
[connect_node-1]     def _initialize_master_working_set():
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 3237, in _call_aside
[connect_node-1]     f(*args, **kwargs)
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 3266, in _initialize_master_working_set
[connect_node-1]     working_set = WorkingSet._build_master()
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 584, in _build_master
[connect_node-1]     ws.require(__requires__)
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 901, in require
[connect_node-1]     needed = self.resolve(parse_requirements(requirements))
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 787, in resolve
[connect_node-1]     raise DistributionNotFound(req, requirers)
[connect_node-1] pkg_resources.DistributionNotFound: The 'python-socketio' distribution was not found and is required by agv-test
[ERROR] [connect_node-1]: process has died [pid 5685, exit code 1, cmd '/home/txgt/ros2_humble/install/agv_test/lib/agv_test/connect_node --ros-args'].

=> 현재 실행 중인 환경(Python)의 site-packages에 python-socketio가 설치되어 있지 않다는 뜻

A. 해결 방법
1) python 사용 확인
- which python3 -> 출력 예시 /usr/bin/python3

2) 필요 패키지 설치 (Humble용 워크스페이스 기준)
- pip3 install python-socketio python-dotenv pyserial

2-1) 패키지 설치 시 오류
- SSLError("bad handshake: ... certificate verify failed") 해당 메세지 출력 시 현재 시스템이 SSL 인증서 문제로 pypi.org와 안전한 연결을 못 하는 상황

2-2) 시간 확인
- date
※ 번 외 data 오류 날 시
- sudo apt install ntpdate
- sudo ntpdate time.google.com

2-3) CA 인증서 최신화
- sudo apt update
- sudo apt install --reinstall ca-certificates
- sudo update-ca-certificates

3) 정상 설치 확인
- pip3 show python-socketio -> 출력 예시 Name: python-socketio ...


+ agv_test라는 Python 패키지를 못 찾는 오류
[connect_node-1] Traceback (most recent call last):
[connect_node-1]   File "/home/txgt/ros2_humble/install/agv_robot_project/lib/agv_robot_project/connect_node", line 11, in <module>
[connect_node-1]     load_entry_point('agv-robot-project', 'console_scripts', 'connect_node')()
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 490, in load_entry_point
[connect_node-1]     return get_distribution(dist).load_entry_point(group, name)
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2854, in load_entry_point
[connect_node-1]     return ep.load()
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2445, in load
[connect_node-1]     return self.resolve()
[connect_node-1]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2451, in resolve
[connect_node-1]     module = __import__(self.module_name, fromlist=['__name__'], level=0)
[connect_node-1]   File "/home/txgt/ros2_humble/build/agv_robot_project/src/agv_robot_project/connect_node.py", line 21, in <module>
[connect_node-1]     runstream_path = os.path.join(package_share_directory, 'scripts', 'runstream.sh')
[connect_node-1] NameError: name 'package_share_directory' is not defined

A. 해결 방법
1) src/agv_test/ 폴더 안에 __init__.py 파일 있는지 확인
- touch src/agv_test/__init__.py

2) setup.py 재확인
- packages=['agv_test'],
- package_dir={'': 'src'},

3) 클린 빌드 후 다시 시작
- rm -rf build install log
- colcon build --packages-select agv_test --symlink-install
- source install/setup.bash


    2. dirve node 오류
[drive_node-2] Traceback (most recent call last):
[drive_node-2]   File "/home/txgt/ros2_humble/install/agv_robot_project/lib/agv_robot_project/drive_node", line 11, in <module>
[drive_node-2]     load_entry_point('agv-robot-project', 'console_scripts', 'drive_node')()
[drive_node-2]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 490, in load_entry_point
[drive_node-2]     return get_distribution(dist).load_entry_point(group, name)
[drive_node-2]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2854, in load_entry_point
[drive_node-2]     return ep.load()
[drive_node-2]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2445, in load
[drive_node-2]     return self.resolve()
[drive_node-2]   File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2451, in resolve
[drive_node-2]     module = __import__(self.module_name, fromlist=['__name__'], level=0)
[drive_node-2]   File "/home/txgt/ros2_humble/build/agv_robot_project/src/agv_robot_project/drive_node.py", line 6, in <module>
[drive_node-2]     from cv_bridge import CvBridge
[drive_node-2] ModuleNotFoundError: No module named 'cv_bridge'

=> cv_bridge는 OpenCV 이미지 ↔ ROS 이미지 메시지 변환에 사용하는 Python 바인딩 패키지인데 해당 패키지가 없거나 경로에 안 잡힌 상태

A. 해결 방법
1) ROS2 Humble에 맞는 cv_bridge 설치
- sudo apt install ros-humble-cv-bridge python3-opencv

* 설치 안될 시 우선 opencv 버전 확인 
- python3 -c "import cv2; print(cv2.__version__)"

-> 버전 값이 3.x ~ 4.x면 문제 없음 다만 cv_bridge 패키지가 없는 것 아래 명령어 대로 설치

- cd ~/ros2_humble/src
- git clone -b humble https://github.com/ros-perception/vision_opencv.git
- cd ~/ros2_humble
- rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
- colcon build --symlink-install
- source install/setup.bash


