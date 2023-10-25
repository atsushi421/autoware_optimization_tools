use std::{
    collections::HashMap,
    fs::{create_dir_all, File},
    io::Write,
};

use serde::Serialize;

use crate::utils::NodeNameConverter;

const OUTPUT_DIR: &str = "complete_node_info";

#[derive(Serialize)]
pub struct CompleteNodeInfo {
    node_name: String,
    package_name: Option<String>,
    plugin_name: Option<String>,
    remappings: Option<HashMap<String, String>>,
    executable: Option<String>,
}

impl CompleteNodeInfo {
    pub fn new(node_name: &str) -> Self {
        Self {
            node_name: node_name.to_string(),
            package_name: None,
            plugin_name: None,
            remappings: None,
            executable: None,
        }
    }

    pub fn set_package_name(&mut self, package_name: &str) {
        if self.package_name.is_some() {
            unreachable!("package_name is already set.");
        }

        self.package_name = Some(package_name.to_string());
    }

    pub fn set_plugin_name(&mut self, plugin_name: &str) {
        if self.plugin_name.is_some() {
            unreachable!("plugin_name is already set.");
        }

        self.plugin_name = Some(plugin_name.to_string());
    }

    pub fn set_remappings(&mut self, remappings: HashMap<String, String>) {
        if self.remappings.is_some() {
            unreachable!("remappings is already set.");
        }

        self.remappings = Some(remappings);
    }

    pub fn exists_package_plugin(&self) -> bool {
        self.package_name.is_some() && self.plugin_name.is_some()
    }

    pub fn get_plugin_name(&self) -> &str {
        self.plugin_name.as_ref().unwrap()
    }

    pub fn set_executable(&mut self, executable: &str) {
        if self.executable.is_some() {
            unreachable!("executable is already set.");
        }

        self.executable = Some(executable.to_string());
    }

    pub fn exists_executable(&self) -> bool {
        self.executable.is_some()
    }
}

pub fn export_complete_node_info(ros_node_name: &str, complete_node_info: &CompleteNodeInfo) {
    create_dir_all(OUTPUT_DIR).unwrap();
    let mut result = File::create(format!(
        "{}/{}.yaml",
        OUTPUT_DIR,
        NodeNameConverter::to_file_name(ros_node_name)
    ))
    .unwrap();
    let yaml_string = serde_yaml::to_string(&complete_node_info).unwrap();
    result
        .write_all(yaml_string.as_bytes())
        .expect("Failed to write to output file");
}
