import os
import subprocess

from tqdm import tqdm  # type: ignore

from utils import NodeNameConverter


def run_command(command):
    result = subprocess.run(command, stdout=subprocess.PIPE, text=True, shell=True)
    return result.stdout


OUTPUT_DIR = 'dynamic_node_info'
SKIP_NODES = ['caret_', 'launch_ros_', 'rviz2', 'rosbag2_player', 'transform_listener_impl_']


def main():
    node_list = run_command('ros2 node list').strip().split('\n')
    for node_name in tqdm(node_list):
        if any(skip_node in node_name for skip_node in SKIP_NODES):
            continue

        info_str = run_command(f'ros2 node info {node_name}')
        info_lines = info_str.strip().split('\n')[1:]
        info_text = '\n'.join(info_lines)

        node_file_name = NodeNameConverter.to_file_name(node_name)
        os.makedirs(f'{OUTPUT_DIR}', exist_ok=True)
        with open(f'{OUTPUT_DIR}/{node_file_name}.yaml', 'w', ) as f:
            f.write(info_text)


if __name__ == "__main__":
    main()
