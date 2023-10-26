use regex::{Regex, RegexBuilder};

pub struct LaunchParseResult {
    pub package: String,
    pub plugin: Option<String>,
    pub executable: Option<String>,
    pub remappings: Option<Vec<(String, String)>>,
}

impl LaunchParseResult {
    fn new(
        package: &str,
        plugin: Option<&str>,
        executable: Option<&str>,
        remappings: Option<Vec<(String, String)>>,
    ) -> Self {
        Self {
            package: package.to_string(),
            plugin: plugin.map(|plugin| plugin.to_string()),
            executable: executable.map(|executable| executable.to_string()),
            remappings,
        }
    }
}

impl PartialEq for LaunchParseResult {
    fn eq(&self, other: &Self) -> bool {
        if self.remappings.is_some() && other.remappings.is_some() {
            self.package == other.package
                && self.plugin == other.plugin
                && self.remappings.clone().unwrap().len() == other.remappings.clone().unwrap().len()
        } else {
            self.package == other.package && self.plugin == other.plugin
        }
    }
}

pub struct LaunchParser {
    launch_py_pattern: Regex,
    launch_py_remapping_pattern: Regex,
    launch_xml_pattern0: Regex,
    launch_xml_pattern1: Regex,
    launch_xml_composable_pattern: Regex,
    launch_xml_remapping_pattern: Regex,
}

impl LaunchParser {
    pub fn new(node_name: &str) -> Self {
        Self {
            launch_py_pattern: RegexBuilder::new(
                &[
                    r#"ComposableNode\("#,
                    r#"\s*package=['"]([^'"]+)['"],"#,
                    r#"(?:\s*namespace=.*?,)?"#,
                    r#"\s*plugin=([^,]+?),"#,
                    r#"\s*name=['"]"#,
                    node_name,
                    r#"['"],"#,
                    r#"(?:\s*remappings=\[(.*?)\],)?"#,
                ]
                .join(""),
            )
            .dot_matches_new_line(true)
            .build()
            .unwrap(),
            launch_py_remapping_pattern: Regex::new(r#"\(([^,]*), ([^)]*)\)"#).unwrap(),
            launch_xml_pattern0: Regex::new(r#"<node pkg="([^"]+)" exec="([^"]+)" name=""#)
                .unwrap(),
            launch_xml_pattern1: Regex::new(r#"<node name=".+?" exec="([^"]+)" pkg="([^"]+)""#)
                .unwrap(),
            launch_xml_composable_pattern: Regex::new(
                &[
                    r#"<composable_node pkg="([^"]+)" plugin="([^"]+)" name=""#,
                    node_name,
                ]
                .join(""),
            )
            .unwrap(),
            launch_xml_remapping_pattern: Regex::new(r#"<remap from="([^"]+)" to="([^"]+)"/>"#)
                .unwrap(),
        }
    }

    fn parse_launch_py_remappings(&self, remappings: &str) -> Vec<(String, String)> {
        let mut remappings_map = Vec::new();
        for cap in self.launch_py_remapping_pattern.captures_iter(remappings) {
            remappings_map.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }
        remappings_map
    }

    pub fn parse_launch_py(&self, launch_py: &str) -> Vec<LaunchParseResult> {
        let mut composable_nodes = Vec::new();
        for cap in self.launch_py_pattern.captures_iter(launch_py) {
            let mut original_remappings = None;
            if let Some(remappings) = cap.get(3) {
                original_remappings = Some(self.parse_launch_py_remappings(remappings.as_str()));
            }

            composable_nodes.push(LaunchParseResult::new(
                cap.get(1).unwrap().as_str(),
                Some(cap.get(2).unwrap().as_str()),
                None,
                original_remappings,
            ));
        }

        composable_nodes
    }

    fn parse_launch_xml_remappings(&self, remappings: &str) -> Option<Vec<(String, String)>> {
        let mut remappings_map = Vec::new();
        for cap in self.launch_xml_remapping_pattern.captures_iter(remappings) {
            remappings_map.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }

        if remappings_map.is_empty() {
            None
        } else {
            Some(remappings_map)
        }
    }

    pub fn parse_candidate_launch_xml(&self, launch_xml: &str) -> Option<LaunchParseResult> {
        self.launch_xml_composable_pattern
            .captures(launch_xml)
            .map(|cap| {
                LaunchParseResult::new(
                    cap.get(1).unwrap().as_str(),
                    Some(cap.get(2).unwrap().as_str()),
                    None,
                    self.parse_launch_xml_remappings(launch_xml),
                )
            })
    }

    pub fn parse_confirmed_launch_xml(&self, launch_xml: &str) -> LaunchParseResult {
        if let Some(cap) = self.launch_xml_pattern0.captures(launch_xml) {
            LaunchParseResult::new(
                cap.get(1).unwrap().as_str(),
                None,
                Some(cap.get(2).unwrap().as_str()),
                self.parse_launch_xml_remappings(launch_xml),
            )
        } else if let Some(cap) = self.launch_xml_pattern1.captures(launch_xml) {
            LaunchParseResult::new(
                cap.get(2).unwrap().as_str(),
                None,
                Some(cap.get(1).unwrap().as_str()),
                self.parse_launch_xml_remappings(launch_xml),
            )
        } else {
            unreachable!()
        }
    }
}
