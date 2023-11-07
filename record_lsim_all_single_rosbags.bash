#!/bin/bash

COMPLETE_NODE_INFO_DIR="$1"

if [ -z "$COMPLETE_NODE_INFO_DIR" ]; then
    echo "Usage: $0 <COMPLETE_NODE_INFO_DIR>"
    exit 1
fi

source /opt/ros/humble/setup.bash

WAITING_TIME_FOR_AUTOWARE_TO_LAUNCH=120
WAITING_TIME_FOR_RECORDER_TO_START=15
WAITING_TIME_FOR_LOCALIZATION=20
WAITING_TIME_FOR_SIMULATION_TO_END=60
GOAL_POSITION_MSG='{header: {frame_id: "map"}, pose: {position: {x: 89516.8, y: 42442.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.840247 , w: 0.542204}}}'

for complete_node_info in "$COMPLETE_NODE_INFO_DIR"/*; do
    if [[ "$node_info" == *"planning-"* ]] || [[ "$node_info" == *"control-"* ]] || [[ "$node_info" == *"system-"* ]] || [[ "$node_info" == *"map-"* ]] || [[ "$node_info" == *"traffic_light"* ]]; then
        continue
    fi

    (setsid ros2 launch autoware_launch logging_simulator.launch.xml map_path:="$HOME/autoware_map/sample-map-rosbag" vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit) &
    autoware_pid=$!

    sleep "$WAITING_TIME_FOR_AUTOWARE_TO_LAUNCH"

    (setsid python3 "$HOME/autoware_optimization_tools/ros2_single_node_replayer/recorder.py" "$complete_node_info") &
    recorder_pid=$!

    sleep "$WAITING_TIME_FOR_RECORDER_TO_START"

    ros2 bag play "$HOME/autoware_map/sample-rosbag/sample.db3" -r 0.5 -s sqlite3 &

    sleep "$WAITING_TIME_FOR_LOCALIZATION"

    ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped "$GOAL_POSITION_MSG" &

    sleep "$WAITING_TIME_FOR_SIMULATION_TO_END"

    kill -SIGINT -- -"$recorder_pid"
    kill -- -"$autoware_pid"
done
