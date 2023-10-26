use serde_yaml::Mapping;
use std::fs::{self, read_to_string};
use walkdir::WalkDir;

use crate::edge_cases::{parse_driver_ros_wrapper_node, parse_gyro_odometer};
use crate::export_node_info::{export_complete_node_info, CompleteNodeInfo};
use crate::fix_remmappings::fix_remappings;
use crate::parse_executable::ExecutableParser;
use crate::parse_launch::LaunchParser;
use crate::parse_plugin::parse_plugin;
use crate::utils::{read_yaml_as_mapping, search_files, NodeNameConverter};

fn get_remapped_topics(mapping: &Mapping, key: &str) -> Vec<String> {
    mapping[key]
        .as_mapping()
        .unwrap()
        .iter()
        .filter_map(|(k, _)| {
            let k_str = k.as_str().unwrap().to_string();
            if k == "/clock"
                || k == "/parameter_events"
                || k == "/rosout"
                || k.as_str().unwrap().contains("debug")
                || k.as_str().unwrap().contains("/tf")
                || k.as_str().unwrap().contains("/diagnostics")
            {
                None
            } else {
                Some(k_str)
            }
        })
        .collect()
}

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
    let mut subs = get_remapped_topics(&dynamic_node_info, "Subscribers");
    let mut pubs = get_remapped_topics(&dynamic_node_info, "Publishers");

    let mut complete_node_info = CompleteNodeInfo::new(&namespace, &node_name);

    // TODO: refactor
    // Edge case
    if node_name.contains("_driver_ros_wrapper_node") {
        let (package_name, plugin_name, executable) = parse_driver_ros_wrapper_node(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_plugin_name(&plugin_name);
        complete_node_info.set_executable(&executable);

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    } else if node_name == "gyro_odometer" {
        let (package_name, executable, mut remappings) = parse_gyro_odometer(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_executable(&executable);
        if let Some(fixed_remappings) = fix_remappings(&mut remappings, &mut subs, &mut pubs) {
            complete_node_info.set_remappings(fixed_remappings);
        }

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    } else if node_name == "occupancy_grid_map_node" {
        let (package_name, plugin_name, mut remappings) =
            crate::edge_cases::parse_occupancy_grid_map_node(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_plugin_name(&plugin_name);
        if let Some(fixed_remappings) = fix_remappings(&mut remappings, &mut subs, &mut pubs) {
            complete_node_info.set_remappings(fixed_remappings);
        }

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

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    } else if node_name == "aggregator_node" {
        let (package_name, executable) = crate::edge_cases::parse_aggregator_node(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_executable(&executable);

        export_complete_node_info(&ros_node_name, &complete_node_info);
        return;
    }

    let launch_parser = LaunchParser::new(&node_name);

    // Parse launch.py
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();

        if file_path.ends_with(".launch.py") {
            let launch_py = fs::read_to_string(file_path).unwrap();
            let composable_nodes = launch_parser.parse_candidate_py(&launch_py);
            if composable_nodes.is_empty() {
                continue;
            }

            let first_composable_node = &composable_nodes[0];
            composable_nodes.iter().for_each(|composable_node| {
                if first_composable_node != composable_node {
                    unreachable!();
                }
            });

            let package_name = &first_composable_node.package;
            let plugin_name = &parse_plugin(
                first_composable_node.plugin.as_ref().unwrap(),
                target_dir,
                &node_name,
            );

            // Cases where there is a match in more than one file
            if complete_node_info.exists_package_plugin() {
                if complete_node_info.get_package_name() == package_name
                    && complete_node_info.get_plugin_name() == plugin_name
                {
                    continue;
                } else {
                    // HACK: Get the component from the remapped topic name and select it if it is included in the namespace
                    let to = &complete_node_info.get_remappings().values().next().unwrap();
                    if to.contains(namespace.split('/').next().unwrap()) {
                        continue;
                    }
                }
            }

            complete_node_info.set_package_name(package_name);
            complete_node_info.set_plugin_name(plugin_name);
            if let Some(original_remappings) = &first_composable_node.remappings {
                let mut remappings_clone = original_remappings.clone();
                if let Some(fixed_remappings) =
                    fix_remappings(&mut remappings_clone, &mut subs, &mut pubs)
                {
                    complete_node_info.set_remappings(fixed_remappings);
                }
            }
        }
    }

    if !complete_node_info.exists_package_plugin() {
        // Search launch.xml
        for entry in WalkDir::new(target_dir).into_iter().flatten() {
            if entry.file_type().is_dir() {
                continue;
            }
            let file_path = entry.path().to_str().unwrap();

            if file_path.ends_with(".launch.xml") {
                let launch_xml = fs::read_to_string(file_path).unwrap();
                if let Some(composable_node) = launch_parser.parse_candidate_xml(&launch_xml) {
                    complete_node_info.set_package_name(&composable_node.package);
                    complete_node_info.set_plugin_name(&composable_node.plugin.unwrap());
                    if let Some(mut original_remappings) = composable_node.remappings {
                        if let Some(fixed_remappings) =
                            fix_remappings(&mut original_remappings, &mut subs, &mut pubs)
                        {
                            complete_node_info.set_remappings(fixed_remappings);
                        }
                    }
                }
            }
        }
    }

    if !complete_node_info.exists_package_plugin() {
        // Search {key_str}.launch.xml
        let mut launch_xml = None;
        let mut key_str = node_name;
        while !key_str.is_empty() {
            let candidate_launch_xmls =
                search_files(target_dir, &format!("{}.launch.xml", key_str));

            match candidate_launch_xmls.len() {
                0 => {
                    key_str = key_str.split('_').collect::<Vec<&str>>()
                        [..key_str.split('_').count() - 1]
                        .join("_");

                    continue;
                }
                1 => {
                    launch_xml = Some(read_to_string(&candidate_launch_xmls[0]).unwrap());
                    break;
                }
                _ => {
                    // Select the launch_xml that contain "</node>".
                    let mut node_contain_count = 0;
                    for candidate in candidate_launch_xmls {
                        let content = read_to_string(candidate).unwrap();
                        if content.contains("</node>") {
                            launch_xml = Some(content);
                            node_contain_count += 1;
                            if node_contain_count > 1 {
                                unreachable!()
                            }
                        }
                    }
                    break;
                }
            }
        }

        if launch_xml.is_none() {
            unreachable!();
        }

        let composable_node = launch_parser.parse_confirmed_xml(&launch_xml.unwrap());

        complete_node_info.set_package_name(&composable_node.package);
        complete_node_info.set_executable(&composable_node.executable.unwrap());
        if let Some(mut original_remappings) = composable_node.remappings {
            if let Some(fixed_remappings) =
                fix_remappings(&mut original_remappings, &mut subs, &mut pubs)
            {
                complete_node_info.set_remappings(fixed_remappings);
            }
        }
    }

    if !complete_node_info.exists_executable() {
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
    }

    if !complete_node_info.exists_executable() {
        panic!("{} was not found.", complete_node_info.get_plugin_name());
    }

    // Export
    export_complete_node_info(&ros_node_name, &complete_node_info)
}
