use indicatif::{ProgressBar, ProgressStyle};
use serde_yaml::Mapping;
use walkdir::WalkDir;

pub struct NodeNameConverter {}

impl NodeNameConverter {
    /// # Examples
    /// ```
    /// use auto_isolated_measurement::utils::NodeNameConverter;
    ///
    /// let file_name = NodeNameConverter::to_file_name("/a/b/c/d_e_f");
    /// assert_eq!(file_name, "a-b-c-d_e_f");
    /// ```
    pub fn to_file_name(node_name: &str) -> String {
        node_name.trim_start_matches('/').replace('/', "-")
    }

    /// # Examples
    /// ```
    /// use auto_isolated_measurement::utils::NodeNameConverter;
    ///
    /// let ros_node_name = NodeNameConverter::to_ros_node_name("a-b-c-d_e_f");
    /// assert_eq!(ros_node_name, "/a/b/c/d_e_f");
    /// ```
    pub fn to_ros_node_name(file_name: &str) -> String {
        format!("/{}", file_name.replace('-', "/"))
    }

    /// # Examples
    /// ```
    /// use auto_isolated_measurement::utils::NodeNameConverter;
    ///
    /// let (namespace, node_name) = NodeNameConverter::to_namespace_and_node_name("/a/b/c/d_e_f");
    /// assert_eq!(namespace, "/a/b/c");
    /// assert_eq!(node_name, "d_e_f");
    /// ```
    pub fn to_namespace_and_node_name(ros_node_name: &str) -> (String, String) {
        let components: Vec<&str> = ros_node_name.split('/').collect();
        (
            components[..components.len() - 1].join("/"),
            components.last().unwrap().to_string(),
        )
    }
}

pub fn create_progress_bar(len: i32) -> ProgressBar {
    let pb = indicatif::ProgressBar::new(len as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}")
            .expect("failed to set progress bar style")
            .progress_chars("##-"),
    );
    pb
}

pub fn read_yaml_as_mapping(path: &str) -> Mapping {
    let file = std::fs::File::open(path).expect("failed to open file");
    let yaml: serde_yaml::Value = serde_yaml::from_reader(file).expect("failed to read yaml");
    yaml.as_mapping().unwrap().clone()
}

pub fn search_files(target_dir: &str, file_name: &str) -> Vec<String> {
    WalkDir::new(target_dir)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|entry| entry.file_name().to_string_lossy() == file_name)
        .map(|entry| entry.path().to_string_lossy().into_owned())
        .collect()
}

pub fn remove_by_str(target: &mut Vec<String>, remove_str: &str) -> String {
    target.remove(
        target
            .iter()
            .position(|value| value.contains(&remove_str))
            .unwrap(),
    )
}
