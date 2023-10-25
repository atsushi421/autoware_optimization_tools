use std::fs::read_to_string;

use regex::Regex;

use crate::utils::search_file;

pub fn parse_map_projection_loader(target_dir: &str) -> (String, String) {
    let launch_file =
        read_to_string(search_file(target_dir, "map_projection_loader.launch.xml")).unwrap();
    let re = Regex::new(r#"pkg="([^"]+)"\s+exec="([^"]+)"#).unwrap();

    let caps = re.captures(&launch_file).unwrap();
    (
        caps.get(1).unwrap().as_str().to_string(),
        caps.get(2).unwrap().as_str().to_string(),
    )
}
