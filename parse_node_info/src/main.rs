use clap::Parser;
use dirs::home_dir;
use parse_node_info::{parse_node_info::parse_node_info, utils::create_progress_bar};
use std::fs::read_dir;

#[derive(Parser)]
#[clap(name = "parse all node info", version = "1.0", about = "")]
struct ArgParser {
    /// Path to dynamic_node_info directory.
    #[clap(
        short = 'd',
        long = "dynamic_node_info_dir",
        default_value = "dynamic_node_info"
    )]
    dynamic_node_info_dir: String,
    /// Path to parsed directory.
    #[clap(short = 'p', long = "parsed_dir", default_value_t = home_dir().unwrap().to_str().unwrap().to_string() + "/autoware/src")]
    parsed_dir: String,
}

fn main() {
    let arg: ArgParser = ArgParser::parse();
    let dynamic_node_infos: Vec<_> = read_dir(arg.dynamic_node_info_dir).unwrap().collect();
    let pb = create_progress_bar(dynamic_node_infos.len() as i32);

    for dynamic_node_info in dynamic_node_infos {
        pb.inc(1);
        parse_node_info(
            dynamic_node_info.unwrap().path().to_str().unwrap(),
            &arg.parsed_dir,
        );
    }
}
