#!/bin/bash

DIR="$1"  # ros2_single_node_replayerの出力ディレクトリ
ISOTED_CPU="$2"  # 実行を制限したいCPUコア番号
TOPIC_NAME="$3"  # 内容を保存したいトピック名

sudo sh -c "echo 0 > /sys/devices/system/cpu/cpufreq/boost"

cd "$DIR"
ROS2_RUN_FILE=$(find "$DIR" -type f -name "ros2_run*")
bash "$ROS2_RUN_FILE" &
PID=$!

taskset --cpu-list -p "$ISOTED_CPU" "$PID"

ROSBAG_FILE=$(find "$DIR" -type f -name "rosbag*")
ros2 bag play "$ROSBAG_FILE" &

if [ -n "$TOPIC_NAME" ]; then
    LOG_FILE_NAME=$(echo "$TOPIC_NAME" | sed 's/^\/\|\/$//g' | tr '/' '_')
    TIMESTAMP=$(date "+%Y%m%d%H%M")
    LOG_FILE_NAME="${TIMESTAMP}_${LOG_FILE_NAME}.csv"
    ros2 topic echo --csv --full-length "$TOPIC_NAME" > "${LOG_FILE_NAME}" &
fi

sleep 180
kill "$PID"

if [ -n "$TOPIC_NAME" ]; then
    echo "Contents of ${TOPIC_NAME}: ${LOG_FILE_NAME}"
fi