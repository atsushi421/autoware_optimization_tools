use std::fs::read_to_string;

use crate::{parse_launch::LaunchParser, utils::search_files};

pub fn parse_occupancy_grid_map_node(_target_dir: &str) -> (String, String, Vec<(String, String)>) {
    let launch_parser = LaunchParser::new("occupancy_grid_map_node");
    let launch_file = read_to_string("/home/atsushi/autoware/src/universe/autoware.universe/perception/probabilistic_occupancy_grid_map/launch/pointcloud_based_occupancy_grid_map.launch.py").unwrap();
    let result = launch_parser
        .parse_candidate_py(&launch_file)
        .pop()
        .unwrap();

    (
        result.package,
        result.plugin.as_ref().unwrap().replace('"', ""),
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

pub fn parse_driver_ros_wrapper_node(
    _target_dir: &str,
) -> (String, String, String, Vec<(String, String)>) {
    // HACK: hard coded, because a variable is used
    // ~/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py
    (
        "nebula_ros".to_string(),                       // package
        "VelodyneDriverRosWrapper".to_string(),         // plugin
        "velodyne_driver_ros_wrapper_node".to_string(), // node name
        vec![
            ("\"aw_points\"".to_string(), "pointcloud_raw".to_string()),
            (
                "\"aw_points_ex\"".to_string(),
                "pointcloud_raw_ex".to_string(),
            ),
        ],
    )
}

pub fn parse_euclidean_cluster(_target_dir: &str) -> (String, String, Vec<(String, String)>) {
    // HACK: hard coded, because a variable is used in the package name.
    // ~/autoware/src/universe/autoware.universe/perception/euclidean_cluster/launch/voxel_grid_based_euclidean_cluster.launch.py
    (
        "euclidean_cluster".to_string(), // package
        "euclidean_cluster::VoxelGridBasedEuclideanClusterNode".to_string(), // plugin
        vec![
            (
                "\"input\"".to_string(),
                "LaunchConfiguration(\"input_pointcloud\")".to_string(),
            ),
            (
                "\"output\"".to_string(),
                "LaunchConfiguration(\"output_clusters\")".to_string(),
            ),
        ],
    )
}
