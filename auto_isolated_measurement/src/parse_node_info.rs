use regex::{Regex, RegexBuilder};
use std::fs;
use walkdir::WalkDir;

use crate::edge_cases::parse_map_projection_loader;
use crate::export_node_info::{export_complete_node_info, CompleteNodeInfo};
use crate::map_remmappings::map_remappings;
use crate::parse_executable::ExecutableParser;
use crate::parse_plugin::parse_plugin;
use crate::utils::{get_remapped_topics_from_mapping, read_yaml_as_mapping, NodeNameConverter};

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

struct LaunchParser {
    launch_py_pattern: Regex,
    launch_py_remapping_pattern: Regex,
    launch_xml_pattern: Regex,
}

impl LaunchParser {
    fn new(node_name: &str) -> Self {
        Self {
            launch_py_pattern: RegexBuilder::new(
                &[
                    r#"ComposableNode\("#,
                    r#"\s*package=['"]([^'"]+)['"],"#,
                    r#"\s*plugin=([^,]+?),"#,
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
            launch_py_remapping_pattern: Regex::new(r#"\(([^,]*), ([^)]*)\)"#).unwrap(),
            launch_xml_pattern: RegexBuilder::new(
                &[
                    r#"ComposableNode\("#,
                    r#"\s*package=['"]([^'"]+)['"],"#,
                    r#"\s*plugin=([^,]+?),"#,
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
        }
    }

    fn parse_remappings(&self, remappings: &str) -> Vec<(String, String)> {
        let mut remappings_map = Vec::new();
        for cap in self.launch_py_remapping_pattern.captures_iter(remappings) {
            remappings_map.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }
        remappings_map
    }

    fn find_composable_nodes(&self, launch_file: &str) -> Vec<ComposableNodeInfo> {
        let mut composable_nodes = Vec::new();
        for cap in self.launch_py_pattern.captures_iter(launch_file) {
            composable_nodes.push(ComposableNodeInfo::new(
                cap.get(1).unwrap().as_str(),
                cap.get(2).unwrap().as_str(),
                self.parse_remappings(cap.get(3).unwrap().as_str()),
            ));
        }

        composable_nodes
    }
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
    let subs = get_remapped_topics_from_mapping(&dynamic_node_info, "Subscribers");
    let pubs = get_remapped_topics_from_mapping(&dynamic_node_info, "Publishers");

    let mut complete_node_info = CompleteNodeInfo::new(&namespace, &node_name);

    // Edge case
    if node_name == "map_projection_loader" {
        let (package_name, executable) = parse_map_projection_loader(target_dir);
        complete_node_info.set_package_name(&package_name);
        complete_node_info.set_executable(&executable);

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
            let composable_nodes = launch_parser.find_composable_nodes(&launch_file);
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
            let plugin_name = &parse_plugin(&first_composable_node.plugin, target_dir, &node_name);

            // TODO: 複数ファイルにマッチがある場合の対応
            if complete_node_info.exists_package_plugin()
                && complete_node_info.get_package_name() == package_name
                && complete_node_info.get_plugin_name() == plugin_name
            {
                continue;
            }

            complete_node_info.set_package_name(&first_composable_node.package);
            complete_node_info.set_plugin_name(&parse_plugin(
                &first_composable_node.plugin,
                target_dir,
                &node_name,
            ));

            if let Some(remappings) = map_remappings(
                first_composable_node.remappings.clone(),
                subs.clone(),
                pubs.clone(),
            ) {
                complete_node_info.set_remappings(remappings);
            }
        }
    }

    if !complete_node_info.exists_package_plugin() {
        unreachable!("{} was not found.", ros_node_name);
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
