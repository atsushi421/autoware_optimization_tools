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
                && self.remappings.as_ref().unwrap().len()
                    == other.remappings.as_ref().unwrap().len()
        } else {
            self.package == other.package && self.plugin == other.plugin
        }
    }
}

pub struct LaunchParser {
    py_candidate_pattern: Regex,
    py_remapping_pattern: Regex,
    xml_confirmed_pattern0: Regex,
    xml_confirmed_pattern1: Regex,
    xml_candidate_pattern: Regex,
    xml_remapping_pattern: Regex,
}

impl LaunchParser {
    pub fn new(node_name: &str) -> Self {
        Self {
            py_candidate_pattern: RegexBuilder::new(
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
            py_remapping_pattern: Regex::new(r#"\(([^,]*), ([^)]*)\)"#).unwrap(),
            xml_confirmed_pattern0: Regex::new(r#"<node pkg="([^"]+)" exec="([^"]+)" name=""#)
                .unwrap(),
            xml_confirmed_pattern1: Regex::new(r#"<node name=".+?" exec="([^"]+)" pkg="([^"]+)""#)
                .unwrap(),
            xml_candidate_pattern: Regex::new(
                &[
                    r#"<composable_node pkg="([^"]+)" plugin="([^"]+)" name=""#,
                    node_name,
                ]
                .join(""),
            )
            .unwrap(),
            xml_remapping_pattern: Regex::new(r#"<remap from="([^"]+)" to="([^"]+)"/>"#).unwrap(),
        }
    }

    fn parse_py_remappings(&self, remappings_str: &str) -> Vec<(String, String)> {
        let mut remappings = Vec::new();
        for cap in self.py_remapping_pattern.captures_iter(remappings_str) {
            remappings.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }
        remappings
    }

    pub fn parse_candidate_py(&self, launch_py: &str) -> Vec<LaunchParseResult> {
        let mut composable_nodes = Vec::new();
        for cap in self.py_candidate_pattern.captures_iter(launch_py) {
            let original_remappings = cap
                .get(3)
                .map(|remappings| self.parse_py_remappings(remappings.as_str()));

            composable_nodes.push(LaunchParseResult::new(
                cap.get(1).unwrap().as_str(),
                Some(cap.get(2).unwrap().as_str()),
                None,
                original_remappings,
            ));
        }

        composable_nodes
    }

    fn parse_xml_remappings(&self, remappings_str: &str) -> Option<Vec<(String, String)>> {
        let mut remappings = Vec::new();
        for cap in self.xml_remapping_pattern.captures_iter(remappings_str) {
            remappings.push((
                cap.get(1).unwrap().as_str().to_string(),
                cap.get(2).unwrap().as_str().to_string(),
            ));
        }

        if remappings.is_empty() {
            None
        } else {
            Some(remappings)
        }
    }

    pub fn parse_candidate_xml(&self, launch_xml: &str) -> Option<LaunchParseResult> {
        self.xml_candidate_pattern.captures(launch_xml).map(|cap| {
            LaunchParseResult::new(
                cap.get(1).unwrap().as_str(),
                Some(cap.get(2).unwrap().as_str()),
                None,
                self.parse_xml_remappings(launch_xml),
            )
        })
    }

    pub fn parse_confirmed_xml(&self, launch_xml: &str) -> LaunchParseResult {
        if let Some(cap) = self.xml_confirmed_pattern0.captures(launch_xml) {
            LaunchParseResult::new(
                cap.get(1).unwrap().as_str(),
                None,
                Some(cap.get(2).unwrap().as_str()),
                self.parse_xml_remappings(launch_xml),
            )
        } else if let Some(cap) = self.xml_confirmed_pattern1.captures(launch_xml) {
            LaunchParseResult::new(
                cap.get(2).unwrap().as_str(),
                None,
                Some(cap.get(1).unwrap().as_str()),
                self.parse_xml_remappings(launch_xml),
            )
        } else {
            unreachable!()
        }
    }
}
