import argparse
import os
import re
import subprocess
from typing import Any

import yaml


def parse_static_info(plugin_name: str, target_dir: str) -> (str, str, str, dict[str, str]):
    COMPOSABLE_NODE_PATTERN = re.compile(
        r"ComposableNode\("
        r"\s*package=['\"]([^'\",]+)['\"],"
        r"\s*plugin=['\"]" + re.escape(plugin_name) + "['\"],"
        r"\s*name=['\"]([^'\",]+)['\"],"
        r"\s*remappings=\[(.*?)\],",
        re.S,
    )
    REMAPPING_PATTERN = re.compile(r'\(([^,]*), ([^)]*)\)')
    COMPONENTS_REGISTER_PATTERN = re.compile(
        r"rclcpp_components_register_node\("
        r"\s*([^\s]+)\s*"
        r"PLUGIN\s*\"" + re.escape(plugin_name) +
        r"\"\s*EXECUTABLE\s*([^\s]+)\s*\)"
    )

    package_name = node_name = ''
    for root, _, files in os.walk(target_dir):
        for file in files:
            if file.endswith('.launch.py'):
                with open(os.path.join(root, file), 'r') as f:
                    match = COMPOSABLE_NODE_PATTERN.search(f.read())
                    if match:
                        if package_name and node_name:
                            print(f"Found multiple ComposableNodes with {plugin_name}")
                            selected_package = ask_user_to_select([package_name, match.group(1)])
                            if selected_package == package_name:
                                continue

                        package_name, node_name, remappings = match.groups()
                        remappings = {m.group(1): m.group(2) for m in REMAPPING_PATTERN.finditer(
                            remappings)} if remappings is not None else []

            elif file == "CMakeLists.txt":
                with open(os.path.join(root, file), 'r') as f:
                    match = COMPONENTS_REGISTER_PATTERN.search(f.read())
                    if match:
                        executable = match.group(2).strip()

    return package_name, node_name, executable, remappings


def parse_dynamic_info(node_name: str) -> (str, list[str]):
    process = subprocess.Popen(
        f"ros2 node list | grep {node_name}", stdout=subprocess.PIPE, shell=True)
    output, _ = process.communicate()
    output_lines = output.decode('utf-8').splitlines()

    if len(output_lines) == 0:
        print(f"No nodes found with {node_name}")
        exit(1)
    elif len(output_lines) == 1:
        ros2_node_name = output_lines[0]
    else:
        print(f'Multiple nodes found with "{node_name}":')
        ros2_node_name = ask_user_to_select(output_lines)

    namespace = ros2_node_name.replace(node_name, '')[:-1]
    process = subprocess.Popen(
        f"ros2 node info {ros2_node_name}", stdout=subprocess.PIPE, shell=True)
    output, _ = process.communicate()
    output_lines = output.decode('utf-8').splitlines()[1:]

    temp_file = f"{node_name}_temp.yaml"
    with open(temp_file, 'w') as f:
        f.write('\n'.join(output_lines))
    with open(temp_file, 'r') as f:
        node_info = yaml.safe_load(f)

    remapped_topics = []
    for input_topic in node_info['Subscribers'].keys():
        if '/clock' in input_topic or '/parameter_events' in input_topic:
            continue
        remapped_topics.append(input_topic)
    for output_topic in node_info['Publishers'].keys():
        if '/rosout' in output_topic or '/parameter_events' in output_topic:
            continue
        remapped_topics.append(output_topic)

    os.remove(temp_file)

    return namespace, remapped_topics


def ask_user_to_select(choice_list: list) -> Any:
    for i, line in enumerate(choice_list):
        print(f"\t[{i + 1}]: {line}")

    while True:
        user_input = input("Enter the number (or 'q' to quit): ").strip()
        if user_input.lower() == 'q':
            break
        try:
            index = int(user_input) - 1
            if 0 <= index < len(choice_list):
                return choice_list[index]
            else:
                print("Invalid number. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--plugin_name', help='The name of the plugin.', required=True)
    parser.add_argument(
        '-d', '--dir', help='The directory to search.',
        default=f"{os.environ['HOME']}/autoware"
    )
    args = parser.parse_args()

    # Static analysis
    package_name, node_name, executable, remappings = parse_static_info(
        args.plugin_name, args.dir)

    # Dynamic analysis
    print("Please run autoware. Press enter to continue...")
    input()
    namespace, remapped_topics = parse_dynamic_info(node_name)

    # Fix remappings
    fixed_remappings = {}
    for original, remapped in remappings.items():
        fixed_original = original.replace('"', '')
        if remapped.startswith('"'):
            fixed_remampped = remapped.replace('"', '')
        else:  # e.g., LaunchConfiguration(...)
            if len(remapped_topics) == 1:
                fixed_remampped = remapped_topics[0]
            else:
                print(f'Please select the remapping for "{fixed_original}":')
                fixed_remampped = ask_user_to_select(remapped_topics)
                remapped_topics.remove(fixed_remampped)
        fixed_remappings[fixed_original] = fixed_remampped

    # Export to yaml
    output_path = f"output/{namespace.strip('/').replace('/', '_')}_{node_name}"
    os.makedirs(output_path, exist_ok=True)
    with open(f"{output_path}/basic_info.yaml", 'w') as f:
        yaml.dump(
            {
                'namespace': namespace,
                'package_name': package_name,
                'executable': executable,
                'node_name': node_name
            }, f, default_flow_style=False
        )
    with open(f"{output_path}/remappings.yaml", 'w') as f:
        yaml.dump(fixed_remappings, f, default_flow_style=False)


if __name__ == '__main__':
    main()
