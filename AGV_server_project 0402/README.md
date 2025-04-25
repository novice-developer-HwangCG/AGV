SERVER 구상도

server_project/
 - rtmp/  nginx.conf          # 카메라 스트림
- docker-compose.yml
- Dockerfile
- redis_db.py
- static.py
- common.py
- requirements.txt
- server.py

실행 방법 - 개발 단계
- docker-compose up -d
- SERVER_RTMP=rtmp://192.168.0.80:1935 REDIS_PASSWORD=agvtestserverproject0402 python3 server.py
