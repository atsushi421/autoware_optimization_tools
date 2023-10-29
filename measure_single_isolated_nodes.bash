#!/bin/bash

REPLAYER_OUTPUT_DIR=${1:-"$HOME/autoware_optimization_tools/ros2_single_node_replayer/output"}
ISOTED_CPU=${2:-"2"}
STORED_TOPIC_NAME="$3"  # optional

# sudo sh -c "echo 0 > /sys/devices/system/cpu/cpufreq/boost"

# Setting for CARET
ulimit -n 16384
source "$HOME/ros2_caret_ws/setenv_caret.bash"
source "$HOME/autoware/install/local_setup.bash"
source "$HOME/autoware/caret_topic_filter.bash"
mkdir -p "$HOME/autoware_optimization_tools/caret_trace_data"
export ROS_TRACE_DIR="$HOME/autoware_optimization_tools/caret_trace_data"

WAITING_TIME_FOR_NODE_TO_START=1
WAITING_TIME_FOR_CARET_TO_START=3
WAITING_TIME_FOR_SIMULATION_TO_END=150

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
    caret_session_name="${node_replay_dir##*/}"
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