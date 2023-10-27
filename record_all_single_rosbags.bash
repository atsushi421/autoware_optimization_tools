#!/bin/bash

complete_node_info_dir="$1"

if [ -z "$complete_node_info_dir" ]; then
  echo "Usage: $0 <complete_node_info_dir>"
  exit 1
fi

for file in "$complete_node_info_dir"/*; do
  echo "$file"

  (setsid ros2 launch autoware_launch logging_simulator.launch.xml map_path:="$HOME/autoware_map/sample-map-rosbag" vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit) &
  pid1=$!

  sleep 110

  (setsid python3 "$HOME/autoware_optimization_tools/ros2_single_node_replayer/recorder.py" "$file") &
  pid2=$!

  sleep 15

  ros2 bag play "$HOME/autoware_map/sample-rosbag/sample.db3" -r 0.2 -s sqlite3 &

  sleep 20

  ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 89516.8, y: 42442.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.840247 , w: 0.542204}}}' &

  sleep 160

  kill -- -"$pid2"

  kill -- -"$pid1"

done
