use regex::Regex;

pub struct ExecutableParser {
    component_register_pattern: Regex,
    variable_pattern: Regex,
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
            variable_pattern: Regex::new(r#"\$\{(.+?)\}"#).unwrap(),
        }
    }

    pub fn find_executable(&self, cmake_file: &str) -> Option<String> {
        self.component_register_pattern
            .captures(cmake_file)
            .map(|cap| {
                let executable = cap.get(1).unwrap().as_str();
                if let Some(variable_cap) = self.variable_pattern.captures(executable) {
                    let variable_name = variable_cap.get(1).unwrap().as_str();
                    let variable_map = if variable_name == "PROJECT_NAME" {
                        self.find_project_name(cmake_file)
                    } else {
                        self.find_variable_map(cmake_file, variable_name)
                    };
                    executable.replace(&[r#"${"#, variable_name, r#"}"#].join(""), &variable_map)
                } else {
                    executable.to_string()
                }
            })
    }

    fn find_project_name(&self, cmake_file: &str) -> String {
        let re = Regex::new(r#"project\((.+?)\)"#).unwrap();
        re.captures(cmake_file)
            .unwrap()
            .get(1)
            .unwrap()
            .as_str()
            .to_string()
    }

    fn find_variable_map(&self, cmake_file: &str, variable_name: &str) -> String {
        let re = Regex::new(&[r#"set\("#, variable_name, r#" (.+?)\)"#].join("")).unwrap();
        re.captures(cmake_file)
            .unwrap()
            .get(1)
            .unwrap()
            .as_str()
            .to_string()
    }
}
