use std::fs::{self, read_to_string};
use walkdir::WalkDir;

use crate::edge_cases::{parse_driver_ros_wrapper_node, parse_gyro_odometer};
use crate::export_node_info::{export_complete_node_info, CompleteNodeInfo};
use crate::map_remmappings::map_remappings;
use crate::parse_executable::ExecutableParser;
use crate::parse_launch::LaunchParser;
use crate::parse_plugin::parse_plugin;
use crate::utils::{
    get_remapped_topics_from_mapping, read_yaml_as_mapping, search_files, NodeNameConverter,
};

pub fn parse_node_info(dynamic_node_info_path: &str, target_dir: &str) {
    let ros_node_name = NodeNameConverter::to_ros_node_name(
        dynamic_node_info_path
            .split('/')
            .collect::<Vec<&str>>()
            .last()
            .unwrap()
            .trim_end_matches(".yaml"),
    );
    let (namespace, node_name) = NodeNameConverter::to_namespace_and_node_name(&ros_node_name);
    let dynamic_node_info = read_yaml_as_mapping(dynamic_node_info_path);
    let subs = get_remapped_topics_from_mapping(&dynamic_node_info, "Subscribers");
    let pubs = get_remapped_topics_from_mapping(&dynamic_node_info, "Publishers");

    let mut complete_node_info = CompleteNodeInfo::new(&namespace, &node_name);

    // Edge case
    if node_name.contains("_driver_ros_wrapper_node") {
        let (package_name, plugin_name, executable) = parse_driver_ros_wrapper_node(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_plugin_name(&plugin_name);
        complete_node_info.set_executable(&executable);

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    } else if node_name == "gyro_odometer" {
        let (package_name, executable, remappings) = parse_gyro_odometer(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_executable(&executable);
        if let Some(fixed_remappings) =
            map_remappings(remappings.clone(), subs.clone(), pubs.clone())
        {
            complete_node_info.set_remappings(fixed_remappings);
        }

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    } else if node_name == "occupancy_grid_map_node" {
        let (package_name, plugin_name, remappings) =
            crate::edge_cases::parse_occupancy_grid_map_node(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_plugin_name(&plugin_name);
        if let Some(fixed_remappings) =
            map_remappings(remappings.clone(), subs.clone(), pubs.clone())
        {
            complete_node_info.set_remappings(fixed_remappings);
        }

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    }

    let launch_parser = LaunchParser::new(&node_name);

    // Parse launch files to get package name, plugin name, and remappings
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();

        if file_path.ends_with(".launch.py") {
            let launch_file = fs::read_to_string(file_path).unwrap();
            let composable_nodes = launch_parser.parse_launch_py(&launch_file);
            if composable_nodes.is_empty() {
                continue;
            }

            let first_composable_node = &composable_nodes[0];
            composable_nodes.iter().for_each(|composable_node| {
                if first_composable_node != composable_node {
                    // HACK: Multiple composable_node matches in the same file,
                    //       only one of which needs to be considered
                    unreachable!();
                }
            });

            let package_name = &first_composable_node.package;
            let plugin_name = &parse_plugin(
                first_composable_node.plugin.as_ref().unwrap(),
                target_dir,
                &node_name,
            );

            // TODO: 複数ファイルにマッチがある場合の対応
            if complete_node_info.exists_package_plugin()
                && complete_node_info.get_package_name() == package_name
                && complete_node_info.get_plugin_name() == plugin_name
            {
                continue;
            }

            complete_node_info.set_package_name(&first_composable_node.package);
            complete_node_info.set_plugin_name(&parse_plugin(
                first_composable_node.plugin.as_ref().unwrap(),
                target_dir,
                &node_name,
            ));

            if let Some(original_remappings) = &first_composable_node.remappings {
                if let Some(fixed_remappings) =
                    map_remappings(original_remappings.clone(), subs.clone(), pubs.clone())
                {
                    complete_node_info.set_remappings(fixed_remappings);
                }
            }
        }
    }

    // Search launch.xml
    if !complete_node_info.exists_package_plugin() {
        let mut launch_xml = None;
        let mut key_str = node_name;
        while !key_str.is_empty() {
            let launch_xmls = search_files(target_dir, &format!("{}.launch.xml", key_str));
            if launch_xmls.is_empty() {
                key_str = key_str.split('_').collect::<Vec<&str>>()
                    [..key_str.split('_').count() - 1]
                    .join("_");

                continue;
            }
            if launch_xmls.len() == 1 {
                launch_xml = Some(read_to_string(&launch_xmls[0]).unwrap());
                break;
            }
            if launch_xmls.len() >= 2 && !key_str.is_empty() {
                unreachable!();
            }
        }

        if launch_xml.is_none() {
            unreachable!();
        }

        let composable_node = launch_parser.parse_launch_xml(&launch_xml.unwrap());

        complete_node_info.set_package_name(&composable_node.package);
        complete_node_info.set_executable(&composable_node.executable.unwrap());
        if let Some(original_remappings) = composable_node.remappings {
            complete_node_info.set_remappings(
                map_remappings(original_remappings.clone(), subs.clone(), pubs.clone()).unwrap(),
            );
        }

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    }

    // Parse CMakeLists.txt to get executable name
    let executable_parser = ExecutableParser::new(complete_node_info.get_plugin_name());
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();
        if file_path.ends_with("CMakeLists.txt") {
            let cmake_file = fs::read_to_string(file_path).unwrap();
            if let Some(executable) = executable_parser.find_executable(&cmake_file) {
                complete_node_info.set_executable(&executable);
                break;
            }
        }
    }

    if !complete_node_info.exists_executable() {
        panic!("{} was not found.", complete_node_info.get_plugin_name());
    }

    // Export
    export_complete_node_info(&ros_node_name, &complete_node_info)
}
