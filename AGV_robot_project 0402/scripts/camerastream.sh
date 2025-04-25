#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_PATH="${SCRIPT_DIR}/../.env"
server_rtmp=""
robot_id=""

# .env 로딩
if [ -f "$ENV_PATH" ]; then
    while IFS='=' read -r key value; do
        key=$(echo "$key" | xargs)
        value=$(echo "$value" | xargs)
        case "$key" in
            "SERVER_RTMP") server_rtmp="$value" ;;
            "ID") robot_id="$value" ;;
        esac
    done < "$ENV_PATH"
else
    echo "[ERROR] Cant find .env file"
    exit 1
fi

# 기본 체크
if [ -z "$server_rtmp" ] || [ -z "$robot_id" ]; then
    echo "[ERROR] SERVER_RTMP or ID value is empty."
    exit 1
fi

final_uri="${server_rtmp}/live/${robot_id}"
echo "[DEBUG] RTMP Addres: $final_uri"

# MJPG → jpegdec → x264 인코딩
gst-launch-1.0 v4l2src device=/dev/video0 ! \
    image/jpeg, width=640, height=480, framerate=30/1 ! \
    jpegdec ! \
    videoconvert ! \
    x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast key-int-max=15 ! \
    video/x-h264, profile=baseline ! \
    queue ! flvmux streamable=true ! \
    rtmpsink location="rtmp://192.168.0.80:1935/live/AGV" sync=false

