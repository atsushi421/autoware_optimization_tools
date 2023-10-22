import pytest

from autoware_optimization_tools.parse_node_info import (parse_dynamic_info,
                                                         parse_static_info)


def test_parse_static_info_normal():
    package_name, node_name, executable, remappings = parse_static_info(
        'DummyComponent', "test/dummy_dirs/normal")
    assert package_name == 'dummy_package'
    assert node_name == 'dummy_node'
    assert executable == 'dummy_exe'
    assert remappings == {'"input"': '"dummy_input"', '"output"': '"dummy_output"'}


def test_parse_static_info_not_str():
    package_name, node_name, executable, remappings = parse_static_info(
        'DummyComponent', "test/dummy_dirs/not_str")
    assert package_name == 'dummy_package'
    assert node_name == 'dummy_node'
    assert executable == 'dummy_exe'
    assert remappings == {
        '"input"': 'LaunchConfiguration("input"', '"output"': 'LaunchConfiguration("output"'}


def test_parse_static_duplication():
    with pytest.raises(OSError):
        parse_static_info('DummyComponent', "test/dummy_dirs/duplication")
