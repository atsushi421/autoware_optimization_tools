use regex::{Regex, RegexBuilder};
use serde::Serialize;
use std::fs;
use std::io::Write;
use std::{collections::HashMap, fs::File};
use walkdir::WalkDir;

use crate::plugin_parser::parse_plugin;
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
        let _i = 1;
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

const OUTPUT_DIR: &str = "complete_node_info";

#[derive(Serialize)]
struct CompleteNodeInfo {
    node_name: String,
    package_name: Option<String>,
    plugin_name: Option<String>,
    remappings: Option<HashMap<String, String>>,
    executable: Option<String>,
}

impl CompleteNodeInfo {
    fn new(node_name: &str) -> Self {
        Self {
            node_name: node_name.to_string(),
            package_name: None,
            plugin_name: None,
            remappings: None,
            executable: None,
        }
    }

    fn set_package_plugin_remappings(
        &mut self,
        package_name: &str,
        plugin_name: &str,
        remappings: HashMap<String, String>,
    ) {
        if self.package_name.is_some() || self.plugin_name.is_some() || self.remappings.is_some() {
            unreachable!("package_name, plugin_name, and remappings are already set.");
        }

        self.package_name = Some(package_name.to_string());
        self.plugin_name = Some(plugin_name.to_string());
        self.remappings = Some(remappings);
    }

    fn exists_package_plugin_remappings(&self) -> bool {
        self.package_name.is_some() && self.plugin_name.is_some() && self.remappings.is_some()
    }

    fn get_plugin_name(&self) -> &str {
        self.plugin_name.as_ref().unwrap()
    }

    fn set_executable(&mut self, executable: &str) {
        if self.executable.is_some() {
            unreachable!("executable is already set.");
        }

        self.executable = Some(executable.to_string());
    }

    fn exists_executable(&self) -> bool {
        self.executable.is_some()
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

    let mut complete_node_info = CompleteNodeInfo::new(&node_name);

    let mut regex_finder = RegexFinder::new(&node_name);

    // Parse launch files to get package name, plugin name, and remappings
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

            let first_composable_node = &composable_nodes[0];
            composable_nodes.iter().for_each(|composable_node| {
                if first_composable_node != composable_node {
                    // HACK: Multiple composable_node matches in the same file,
                    //       only one of which needs to be considered
                    unreachable!();
                }
            });

            complete_node_info.set_package_plugin_remappings(
                &first_composable_node.package,
                &parse_plugin(&first_composable_node.plugin, target_dir, &node_name),
                map_remappings(
                    first_composable_node.remappings.clone(),
                    subs.clone(),
                    pubs.clone(),
                )
                .unwrap(),
            );
        }
    }

    if !complete_node_info.exists_package_plugin_remappings() {
        unreachable!("{} was not found.", node_name);
    }

    // Parse CMakeLists.txt to get executable name
    regex_finder.set_component_register_pattern(complete_node_info.get_plugin_name());
    for entry in WalkDir::new(target_dir).into_iter().flatten() {
        if entry.file_type().is_dir() {
            continue;
        }
        let file_path = entry.path().to_str().unwrap();
        if file_path.ends_with("CMakeLists.txt") {
            let cmake_file = fs::read_to_string(file_path).unwrap();
            if let Some(executable) = regex_finder.find_executable(&cmake_file) {
                complete_node_info.set_executable(&executable);
                break;
            }
        }
    }

    if !complete_node_info.exists_executable() {
        panic!("{} was not found.", complete_node_info.get_plugin_name());
    }

    // Export
    fs::create_dir_all(OUTPUT_DIR).unwrap();
    let mut result = File::create(format!(
        "{}/{}.yaml",
        OUTPUT_DIR,
        NodeNameConverter::to_file_name(&ros_node_name)
    ))
    .unwrap();
    let yaml_string = serde_yaml::to_string(&complete_node_info).unwrap();
    result
        .write_all(yaml_string.as_bytes())
        .expect("Failed to write to output file");
}
