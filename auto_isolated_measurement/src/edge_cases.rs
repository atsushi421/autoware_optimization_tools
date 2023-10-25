use std::fs::read_to_string;

use regex::Regex;

use crate::utils::search_file;

pub fn parse_map_projection_loader(target_dir: &str) -> (String, String) {
    let launch_file =
        read_to_string(search_file(target_dir, "map_projection_loader.launch.xml")).unwrap();
    let re = Regex::new(r#"pkg="([^"]+)"\s+exec="([^"]+)"#).unwrap();

    let caps = re.captures(&launch_file).unwrap();
    (
        caps.get(1).unwrap().as_str().to_string(),
        caps.get(2).unwrap().as_str().to_string(),
    )
}

pub fn parse_topic_state_monitor(target_dir: &str) -> (String, String) {
    let launch_file =
        read_to_string(search_file(target_dir, "topic_state_monitor.launch.xml")).unwrap();
    let re = Regex::new(r#"pkg="([^"]+)" +exec="([^"]+)"#).unwrap();

    let caps = re.captures(&launch_file).unwrap();
    (
        caps.get(1).unwrap().as_str().to_string(),
        caps.get(2).unwrap().as_str().to_string(),
    )
}

pub fn parse_driver_ros_wrapper_node(_target_dir: &str) -> (String, String, String) {
    // HACK
    (
        "nebula_ros".to_string(),
        "VelodyneDriverRosWrapper".to_string(),
        "velodyne_driver_ros_wrapper_node".to_string(),
    )
}
