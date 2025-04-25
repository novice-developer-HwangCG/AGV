Robot 구상도

agv_project/                 # AGV ROS2 패키지
- launch/  agv_launch.py    # 모든 노드 실행
- src/  agv_robot_project/
  - connect_node.py      # PC 서버와 AGV 간 연결 (PC ↔ Robot 통신)
  - drive_node.py        # 라인트레이싱 + 속도 프로파일 (S-curve / Trapezoidal)
  - lidar_node.py        # LIDAR 거리 측정 및 Zero Set
  - ethernet_node.py     # 점검장비와 이더넷 통신
  - watchdog_node.py     # Watchdog 기능 (노드 비정상 종료 시 긴급 정지)
- config/  params.yaml          # 속도 설정, LIDAR 설정 등 필요 X
- scripts/
  - emergency_stop.py    # 긴급 정지 기능 추가
  - camerastream.sh      # 카메라 스트림 기능
- .env
- package.xml
- requirements.txt
- setup.py

src 디렉터리 내에 코드 파일 추가 pico_node.py
- 모터에게 속도 명령 값을 받아 보내는건 pico가 담당 (jetson → pico → motor)

카메라 스트림 방법

server 우선 열기
로봇 카메라 실행 ./scripts/camerastream.sh

server pc에서 아래 명령어 실행
ffplay -fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0 \
  -i rtmp://192.168.0.80:1935/live/AGV


추가 카메라 스트림 오류 날 경우 .env 파일을 우선 확인
- cat -A .env

여기에 '^M' 같은 문자가 보이면 아래 명령어로 문제 없애버리기 -> 윈도우 줄 바꿈 문자라 이상하게 처리됨

- dos2unix .env

dos2unix 없으면 아래 명령어로 설치

- sudo apt-get install dos2unix
