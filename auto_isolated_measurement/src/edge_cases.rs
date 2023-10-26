use std::fs::read_to_string;

use crate::{parse_launch::LaunchParser, utils::search_files};

pub fn parse_occupancy_grid_map_node(_target_dir: &str) -> (String, String, Vec<(String, String)>) {
    let launch_parser = LaunchParser::new("occupancy_grid_map_node");
    let launch_file = read_to_string("/home/atsushi/autoware/src/universe/autoware.universe/perception/probabilistic_occupancy_grid_map/launch/pointcloud_based_occupancy_grid_map.launch.py").unwrap();
    let result = &launch_parser.parse_candidate_py(&launch_file)[0];

    (
        result.package.clone(),
        result.plugin.as_ref().unwrap().replace('"', "").clone(),
        result.remappings.as_ref().unwrap().clone(),
    )
}

pub fn parse_gyro_odometer(_target_dir: &str) -> (String, String, Vec<(String, String)>) {
    let launch_parser = LaunchParser::new("gyro_odometer_node");
    let launch_file = read_to_string("/home/atsushi/autoware/src/universe/autoware.universe/localization/gyro_odometer/launch/gyro_odometer.launch.xml").unwrap();
    let result = launch_parser.parse_confirmed_xml(&launch_file);

    (
        result.package,
        result.executable.unwrap(),
        result.remappings.unwrap(),
    )
}

pub fn parse_aggregator_node(target_dir: &str) -> (String, String) {
    let launch_parser = LaunchParser::new("aggregator_node");
    let launch_file =
        read_to_string(&search_files(target_dir, "system_error_monitor.launch.xml")[0]).unwrap();
    let result = launch_parser.parse_confirmed_xml(&launch_file);

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
