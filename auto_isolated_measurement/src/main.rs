use auto_isolated_measurement::parse_node_info::parse_node_info;
use clap::Parser;
use dirs::home_dir;
use std::fs::read_dir;

#[derive(Parser)]
#[clap(name = "parse node info", version = "1.0", about = "")]
struct ArgParser {
    /// Path to DAGSet directory.
    #[clap(
        short = 'd',
        long = "dymanic_node_info_dir",
        default_value = "dynamic_node_info"
    )]
    dymanic_node_info_dir: String,
    /// Number of processing cores.
    #[clap(short = 'p', long = "parsed_dir", default_value_t = home_dir().unwrap().to_str().unwrap().to_string() + "/autoware/src")]
    parsed_dir: String,
}

fn main() {
    let arg: ArgParser = ArgParser::parse();

    for entry in read_dir(arg.dymanic_node_info_dir).unwrap() {
        let path = entry.unwrap().path();
        parse_node_info(path.to_str().unwrap(), &arg.parsed_dir);
    }
}
