use regex::Regex;

pub struct ExecutableParser {
    component_register_pattern: Regex,
}

impl ExecutableParser {
    pub fn new(plugin_name: &str) -> Self {
        Self {
            component_register_pattern: Regex::new(
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
        }
    }

    pub fn find_executable(&self, cmake_file: &str) -> Option<String> {
        self.component_register_pattern
            .captures(cmake_file)
            .map(|cap| cap.get(1).unwrap().as_str().to_string())
    }
}
