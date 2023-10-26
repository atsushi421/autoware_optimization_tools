use std::fs::read_to_string;

use crate::parse_launch::LaunchParser;
use crate::utils::search_file;

pub fn parse_map_projection_loader(target_dir: &str) -> (String, String) {
    let launch_file =
        read_to_string(search_file(target_dir, "map_projection_loader.launch.xml")).unwrap();
    let launch_parser = LaunchParser::new("map_projection_loader");
    let result = launch_parser.parse_launch_xml(&launch_file);

    (result.package, result.executable.unwrap())
}

pub fn parse_topic_state_monitor(target_dir: &str) -> (String, String) {
    let launch_file =
        read_to_string(search_file(target_dir, "topic_state_monitor.launch.xml")).unwrap();
    let launch_parser = LaunchParser::new("dummy");
    let result = launch_parser.parse_launch_xml(&launch_file);

    (result.package, result.executable.unwrap())
}

pub fn parse_driver_ros_wrapper_node(_target_dir: &str) -> (String, String, String) {
    // HACK
    (
        "nebula_ros".to_string(),
        "VelodyneDriverRosWrapper".to_string(),
        "velodyne_driver_ros_wrapper_node".to_string(),
    )
}
