FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# 기본 패키지 설치
RUN apt update && \
    apt install -y python3 python3-pip libglib2.0-0 libsm6 libxext6 libxrender-dev ffmpeg && \
    apt clean

# 작업 디렉토리 설정
WORKDIR /app

# 전체 프로젝트 복사
COPY . .

# Python 패키지 설치
RUN pip install -r requirements.txt

EXPOSE 5000
# 기본 실행 명령
# CMD "python3 server.py"
