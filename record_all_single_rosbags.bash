#!/bin/bash

complete_node_info_dir="$1"

if [ -z "$complete_node_info_dir" ]; then
  echo "Usage: $0 <complete_node_info_dir>"
  exit 1
fi

WAITING_TIME_FOR_AUTOWARE_TO_LAUNCH=120
WAITING_TIME_FOR_RECORDER_TO_START=15
WAITING_TIME_FOR_LOCALIZATION=20
WAITING_TIME_FOR_SIMULATION_TO_END=160
GOAL_POSITION_MSG='{header: {frame_id: "map"}, pose: {position: {x: 89516.8, y: 42442.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.840247 , w: 0.542204}}}'

for complete_node_info in "$complete_node_info_dir"/*; do
  (setsid ros2 launch autoware_launch logging_simulator.launch.xml map_path:="$HOME/autoware_map/sample-map-rosbag" vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit) &
  autoware_pid=$!

  sleep "$WAITING_TIME_FOR_AUTOWARE_TO_LAUNCH"

  (setsid python3 "$HOME/autoware_optimization_tools/ros2_single_node_replayer/recorder.py" "$complete_node_info") &
  recorder_pid=$!

  sleep "$WAITING_TIME_FOR_RECORDER_TO_START"

  ros2 bag play "$HOME/autoware_map/sample-rosbag/sample.db3" -r 0.2 -s sqlite3 &

  sleep "$WAITING_TIME_FOR_LOCALIZATION"

  ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped "$GOAL_POSITION_MSG" &

  sleep "$WAITING_TIME_FOR_SIMULATION_TO_END"

  kill -- -"$recorder_pid"
  kill -- -"$autoware_pid"
done
