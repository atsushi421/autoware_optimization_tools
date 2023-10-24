use regex::{Regex, RegexBuilder};
use std::collections::HashMap;
use std::fs;
use walkdir::WalkDir;

use auto_isolated_measurement::utils::{
    get_remapped_topics_from_mapping, read_yaml_as_mapping, NodeNameConverter,
};

struct ComposableNodeInfo {
    package: String,
    plugin: String,
    remappings: Vec<(String, String)>,
}

impl ComposableNodeInfo {
    fn new(package: &str, plugin: &str, remappings: Vec<(String, String)>) -> Self {
        Self {
            package: package.to_string(),
            plugin: plugin.to_string(),
            remappings,
        }
    }
}

impl PartialEq for ComposableNodeInfo {
    fn eq(&self, other: &Self) -> bool {
        self.package == other.package
            && self.plugin == other.plugin
            && self.remappings.len() == other.remappings.len()
    }
}

struct RegexFinder {
    composable_node_pattern: Regex,
    remapping_pattern: Regex,
    component_register_pattern: Option<Regex>,
}

impl RegexFinder {
    fn new(node_name: &str) -> Self {
        Self {
            composable_node_pattern: RegexBuilder::new(
                &[
                    r#"ComposableNode\("#,
                    r#"\s*package=['"]([^'"]+)['"],"#,
                    r#"\s*plugin=['"]([^'"]+)['"],"#,
                    r#"\s*name=['"]"#,
                    node_name,
                    r#"['"],"#,
                    r#"\s*remappings=\[(.*?)\],"#,
                ]
                .join(""),
            )
            .dot_matches_new_line(true)
            .build()
            .unwrap(),
            remapping_pattern: Regex::new(r#"\(([^,]*), ([^)]*)\)"#).unwrap(),
            component_register_pattern: None,
        }
    }

    fn parse_remappings(&self, remappings: &str) -> Vec<(String, String)> {
        let mut remappings_map = Vec::new();
        for cap in self.remapping_pattern.captures_iter(remappings) {
            remappings_map.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }
        remappings_map
    }

    fn find_composable_nodes(&self, launch_file: &str) -> Vec<ComposableNodeInfo> {
        let mut composable_nodes = Vec::new();
        for cap in self.composable_node_pattern.captures_iter(launch_file) {
            composable_nodes.push(ComposableNodeInfo::new(
                cap.get(1).unwrap().as_str(),
                cap.get(2).unwrap().as_str(),
                self.parse_remappings(cap.get(3).unwrap().as_str()),
            ));
        }

        composable_nodes
    }

    fn set_component_register_pattern(&mut self, plugin_name: &str) {
        self.component_register_pattern = Some(
            Regex::new(
                &[
                    r#"rclcpp_components_register_node\("#,
                    r#"\s*[^\s]+\s*"#,
                    r#"PLUGIN\s*""#,
                    plugin_name,
                    r#"""#,
                    r#"\s*EXECUTABLE\s*([^\s]+)\s*\)"#,
                ]
                .join(""),
            )
            .unwrap(),
        );
    }

    fn find_executable(&self, cmake_file: &str) -> Option<String> {
        self.component_register_pattern
            .as_ref()
            .unwrap()
            .captures(cmake_file)
            .map(|cap| cap.get(1).unwrap().as_str().to_string())
    }
}

fn map_remappings(
    mut original_remappings: Vec<(String, String)>,
    mut subs: Vec<String>,
    mut pubs: Vec<String>,
) -> Option<HashMap<String, String>> {
    let mut fixed_remappings = HashMap::new();

    // First, the topics for which the remapping string is directly specified are mapped.
    original_remappings.retain(|(from, to)| {
        if to.starts_with('\"') {
            fixed_remappings.insert(from.replace('\"', ""), to.replace('\"', ""));
            subs.retain(|sub_| sub_ != to);
            pubs.retain(|pub_| pub_ != to);
            false
        } else {
            true
        }
    });

    // If there is one input and one output, they correspond to sub and pub respectively.
    if original_remappings.len() <= 2 && subs.len() == 1 && pubs.len() == 1 {
        for (from, _) in original_remappings {
            if from.contains("input") {
                fixed_remappings.insert(from.replace('\"', ""), subs[0].clone());
            } else if from.contains("output") {
                fixed_remappings.insert(from.replace('\"', ""), pubs[0].clone());
            }
        }
    }

    // TODO: support more cases

    if fixed_remappings.is_empty() {
        None
    } else {
        Some(fixed_remappings)
    }
}

fn parse_node_info(
    dynamic_node_info_path: &str,
    target_dir: &str,
) -> (String, String, HashMap<String, String>, String) {
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

    let mut regex_finder = RegexFinder::new(&node_name);

    // Parse launch files to get package name, plugin name, and remappings
    let mut package_name: Option<String> = None;
    let mut plugin_name: Option<String> = None;
    let mut remappings: Option<HashMap<String, String>> = None;
    let mut found_flag = false;
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();
        if file_path.ends_with(".launch.py") {
            // filter by namespace
            if !namespace.split('/').collect::<Vec<&str>>()[1..]
                .iter()
                .any(|ns| file_path.contains(ns))
            {
                continue;
            }

            let launch_file = fs::read_to_string(file_path).unwrap();
            let composable_nodes = regex_finder.find_composable_nodes(&launch_file);
            if composable_nodes.is_empty() {
                continue;
            }

            if found_flag {
                unreachable!(); // HACK
            }

            let first_composable_node = &composable_nodes[0];
            composable_nodes.iter().for_each(|composable_node| {
                if first_composable_node != composable_node {
                    // HACK: Multiple composable_node matches in the same file,
                    //       only one of which needs to be considered
                    unreachable!();
                }
            });

            package_name = Some(first_composable_node.package.clone());
            plugin_name = Some(first_composable_node.plugin.clone());
            remappings = Some(
                map_remappings(
                    first_composable_node.remappings.clone(),
                    subs.clone(),
                    pubs.clone(),
                )
                .unwrap(),
            );
            found_flag = true;
        }
    }

    if package_name.is_none() || plugin_name.is_none() || remappings.is_none() {
        panic!("{} was not found.", node_name);
    }

    // Parse CMakeLists.txt to get executable name
    regex_finder.set_component_register_pattern(plugin_name.as_ref().unwrap());
    let mut executable: Option<String> = None;
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();
        if file_path.ends_with("CMakeLists.txt") {
            let cmake_file = fs::read_to_string(file_path).unwrap();
            if let Some(executable_) = regex_finder.find_executable(&cmake_file) {
                executable = Some(executable_);
                break;
            }
        }
    }

    if executable.is_none() {
        panic!("{} was not found.", plugin_name.unwrap());
    }

    (
        package_name.unwrap(),
        plugin_name.unwrap(),
        remappings.unwrap(),
        executable.unwrap(),
    )
}

fn main() {
    let (package_name, plugin_name, remappings, executable) = parse_node_info("/home/atsushi/autoware_optimization_tools/auto_isolated_measurement/dynamic_node_info/perception-object_recognition-detection-voxel_grid_downsample_filter.yaml",
     "/home/atsushi/autoware/src"
    );

    print!(
        "package: {}\nplugin: {}\nremappings: ",
        package_name, plugin_name
    );

    // parse(
    //     "/home/atsushi/autoware_optimization_tools/auto_isolated_measurement/dynamic_node_info/sensing-lidar-concatenate_data.yaml",
    //     "/home/atsushi/autoware/src"
    // )
}
