#!/bin/bash

REPLAYER_OUTPUT_DIR=${1:-"$HOME/autoware_optimization_tools/ros2_single_node_replayer/output"}
ISOTED_CPU=${2:-"2"}
OUTPUT_DIR=${3:-"$HOME/autoware_optimization_tools/caret_trace_data"}
STORED_TOPIC_NAME="$4"  # optional

# Setting for CARET
ulimit -n 65535
ulimit -c unlimited
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
source "$HOME/autoware/install/local_setup.bash"
source ./caret_topic_filter.bash
mkdir -p "$OUTPUT_DIR"
export ROS_TRACE_DIR="$OUTPUT_DIR"

WAITING_TIME_FOR_NODE_TO_START=1
WAITING_TIME_FOR_CARET_TO_START=3
WAITING_TIME_FOR_SIMULATION_TO_END=60

for node_replay_dir in "$REPLAYER_OUTPUT_DIR"/*; do
    cd "$node_replay_dir"

    # Run node
    ros2_run_file=$(find "$node_replay_dir" -type f -name "ros2_run*")
    (setsid bash "$ros2_run_file") &
    ros2_run_pid=$!

    sleep "$WAITING_TIME_FOR_NODE_TO_START"

    # Set affinity
    pids_include_children=$(ps -o pid= -s $ros2_run_pid)
    for child_pid in $pids_include_children; do
        cmd=$(ps -o cmd -p "$child_pid")
        if [[ $cmd == *"install"* ]]; then
            taskset --cpu-list -p "$ISOTED_CPU" "$child_pid"
        fi
    done

    # Start CARET
    current_time=$(date +"%Y%m%d-%H%M%S")
    caret_session_name="${current_time}_from_${node_replay_dir##*/}"
    (setsid ros2 caret record --immediate -s "$caret_session_name") &
    caret_pid=$!

    sleep "$WAITING_TIME_FOR_CARET_TO_START"

    # Play rosbag
    rosbag_dir=$(find "$node_replay_dir" -type d -name "rosbag*")
    ros2 bag play "$rosbag_dir" &

    # (optional) Store topic
    if [ -n "$STORED_TOPIC_NAME" ]; then
        log_file_name=$(echo "$STORED_TOPIC_NAME" | sed 's/^\/\|\/$//g' | tr '/' '_')
        timestamp=$(date "+%Y%m%d%H%M")
        log_file_name="${timestamp}_${log_file_name}.csv"
        ros2 topic echo --csv --full-length "$STORED_TOPIC_NAME" > "${log_file_name}" &
        # TODO: kill this process
    fi

    # Wait for simulation to end and kill node
    sleep "$WAITING_TIME_FOR_SIMULATION_TO_END"
    kill -SIGINT -- -"$caret_pid"
    kill -- -"$ros2_run_pid"

    if [ -n "$STORED_TOPIC_NAME" ]; then
        echo "Stored contents of ${STORED_TOPIC_NAME} in ${log_file_name}"
    fi
done