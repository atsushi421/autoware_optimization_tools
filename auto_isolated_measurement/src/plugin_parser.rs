use regex::Regex;
use walkdir::WalkDir;

use crate::utils::read_yaml_as_mapping;

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

        let param_yaml = WalkDir::new(target_dir)
            .into_iter()
            .filter_map(Result::ok)
            .find(|entry| {
                entry.file_name().to_str() == Some(&format!("{}.param.yaml", plugin_param))
            })
            .map(|entry| entry.path().to_string_lossy().into_owned())
            .unwrap();

        let param_yaml = read_yaml_as_mapping(&param_yaml);
        param_yaml["/**"]["ros__parameters"][node_name]["plugin"]
            .as_str()
            .unwrap()
            .to_string()
    }
}
