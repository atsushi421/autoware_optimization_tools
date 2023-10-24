use std::fs::{self, File};
use std::io::Write;
use crate::utils::{run_command, create_progress_bar, NodeNameConverter};

const OUTPUT_DIR: &str = "dynamic_node_info";
const SKIP_NODES: [&str; 5] = ["caret_", "launch_ros_", "rviz2", "rosbag2_player", "transform_listener_impl_"];

pub fn save_dynamic_node_info() {
    let node_list_str = run_command("ros2 node list");
    let node_list = node_list_str.trim().split('\n');
    let pb = create_progress_bar(node_list.clone().count() as i32);
    for ros_node_name in node_list {
        pb.inc(1);
        if SKIP_NODES.iter().any(|&skip_node| ros_node_name.contains(skip_node)) {
            continue;
        }

        let info_str = run_command(&format!("ros2 node info {}", ros_node_name));
        let info_lines: Vec<&str> = info_str.trim().split('\n').collect();
        let info_text = info_lines[1..].join("\n");

        fs::create_dir_all(OUTPUT_DIR).unwrap();
        let mut file = File::create(format!("{}/{}.yaml", OUTPUT_DIR, NodeNameConverter::to_file_name(ros_node_name))).unwrap();
        file.write_all(info_text.as_bytes()).unwrap();
    }
}
