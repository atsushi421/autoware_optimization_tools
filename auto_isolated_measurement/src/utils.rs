use std::process::Command;
use indicatif::{ProgressBar, ProgressStyle};

pub struct NodeNameConverter {}

impl NodeNameConverter {
    pub fn to_file_name(node_name: &str) -> String {
        node_name.trim_start_matches('/').replace('/', "-")
    }

    pub fn to_ros_node_name(file_name: &str) -> String {
        format!("/{}", file_name.replace('-', "/"))
    }

    // pub fn to_namespace_and_node_name(ros_node_name: &str) -> (String, String) {
    //     unimplemented!()
    // }
}

pub fn run_command(command: &str) -> String {
    let output = Command::new("sh")
        .arg("-c")
        .arg(command)
        .output()
        .expect("failed to execute command");
    String::from_utf8(output.stdout).unwrap()
}

pub fn create_progress_bar(len: i32) -> ProgressBar {
    let pb = indicatif::ProgressBar::new(len as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}")
            .expect("failed to set progress bar style")
            .progress_chars("##-"),
    );
    pb
}