import argparse
import os
import re
import subprocess
from typing import Tuple

import yaml
from tqdm import tqdm  # type: ignore


def get_original_pubs_and_subs(
        trace_data: str, dynamic_node_info_dir: str
) -> Tuple[dict[str, str], dict[str, str]]:
    trace_dir_name = trace_data.split('/')[0]
    date_pattern = r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}_"
    node_name = re.sub(date_pattern, '', trace_dir_name).replace('__', '-')

    with open(f"{dynamic_node_info_dir}/{node_name}.yaml", 'r') as f:
        dynamic_node_info = yaml.safe_load(f)
        pubs = dynamic_node_info['Publishers']
        subs = dynamic_node_info['Subscribers']

    return pubs, subs


def main(trace_data_dir: str, dynamic_node_info_dir: str) -> None:
    for trace_data in tqdm(os.listdir(trace_data_dir)):
        output = subprocess.run(
            f"ros2 caret topic_summary {trace_data_dir}/{trace_data}",
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, shell=True
        )

        if 'events found' not in output.stdout:
            print(f"\n|{trace_data}|\n Measurement by CARET has failed.")
            continue

        pubs, subs = get_original_pubs_and_subs(trace_data, dynamic_node_info_dir)
        for original_sub_topic in subs.keys():
            if 'debug' in original_sub_topic:
                continue
            if original_sub_topic not in output.stdout:
                print(f"\n|{trace_data}|\n Subscriber topic {original_sub_topic} not found.")

        for original_pub_topic in pubs.keys():
            if 'debug' in original_pub_topic:
                continue
            if original_pub_topic not in output.stdout:
                print(f"\n|{trace_data}|\n Publisher topic {original_pub_topic} not found.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Check result of measure_single_isolated_nodes.bash")
    parser.add_argument(
        "trace_data_dir", nargs='?',
        default=f"{os.environ['HOME']}/autoware_optimization_tools/caret_trace_data", help="")
    parser.add_argument(
        "dynamic_node_info_dir", nargs='?',
        default=f"{os.environ['HOME']}/autoware_optimization_tools/parse_node_info/dynamic_node_info",
        help="")

    args = parser.parse_args()
    main(args.trace_data_dir, args.dynamic_node_info_dir)
