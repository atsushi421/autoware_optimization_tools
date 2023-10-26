use regex::Regex;

use crate::utils::{read_yaml_as_mapping, search_files};

pub fn parse_plugin(plugin_name: &str, target_dir: &str, node_name: &str) -> String {
    if plugin_name.starts_with('\"') {
        plugin_name.replace('\"', "")
    } else {
        // get plugin name from param.yaml
        let plugin_param_pattern = Regex::new(r#"self\.(\w+)_param\["\w+"\]\["plugin"\]"#).unwrap();
        let plugin_param = plugin_param_pattern
            .captures(plugin_name)
            .unwrap()
            .get(1)
            .unwrap()
            .as_str();

        let param_yaml = &search_files(target_dir, &format!("{}.param.yaml", plugin_param))[0];
        let param_map = read_yaml_as_mapping(&param_yaml);
        param_map["/**"]["ros__parameters"][node_name]["plugin"]
            .as_str()
            .unwrap()
            .to_string()
    }
}
