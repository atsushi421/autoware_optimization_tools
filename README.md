# autoware_optimization_tools

## 1. parse_node_info
This tool automatically parses the information required for the [ros2_single_node_replayer](https://github.com/sykwer/ros2_single_node_replayer) input in the following steps:

```bash
# Please launch Autoware on another terminal (e.g., rosbag replay simulation: https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/).

$ cd parse_node_info
$ cargo run --release --bin save_dynamic_node_info  # Save the dynamic information of all nodes
$ cargo run --release  # Parse the complete information of all nodes on the basis of dynamic and static information
```

## 2. record_single_rosbags
The following command generates rosbags and binaries for single-node execution for all nodes using the output of [1. parse_node_info](#1-parse_node_info). It takes approximately $220 \text{[s]} \times \text{number of nodes}$ to complete the command.

```bash
$ bash record_lsim_all_single_rosbags.bash
```

## 3. measure_single_isolated_nodes
First, prepare for a isolated environment by referring to [this page](https://tier4.atlassian.net/wiki/spaces/~5ed0b4584824b20c18371c06/pages/2553119448). Once preparations are complete, the following commands allow measurements  in a single-node isolated environment for all nodes. Measurements are made using [CARET](https://tier4.github.io/caret_doc/latest/). It takes approximately $65 \text{[s]} \times \text{number of nodes}$ to complete the commands.

```bash
$ sudo bash fix_frequencies.bash
$ bash measure_single_isolated_nodes.bash
```

## compare_topics
TBD
