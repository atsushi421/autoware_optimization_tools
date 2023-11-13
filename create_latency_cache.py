import argparse
import os
import shutil
from abc import ABCMeta

import numpy as np
from caret_analyze import (Application, Architecture, Lttng,  # type: ignore
                           LttngEventFilter)
from caret_analyze.record import Latency  # type: ignore
from caret_analyze.runtime import Node  # type: ignore


class Common(metaclass=ABCMeta):

    def __init__(self, trace_dir: str, cropped_offset: int, cropped_duration: int, output_dir: str) -> None:
        self.trace_dir = trace_dir
        self.cropped_offset = cropped_offset
        self.cropped_duration = cropped_duration
        self.output_dir = output_dir

    @staticmethod
    def is_lsim_target(node_name: str) -> bool:
        if 'sensing' not in node_name and 'localization' not in node_name and 'perception' not in node_name:
            return False
        if "default_ad" in node_name or 'traffic_light' in node_name or "pose_initializer" in node_name or 'dpc' in node_name:
            return False

        return True

    def create_cbs_cache(self, node: Node) -> None:
        for cb in node.callbacks:
            cb_latencies = Latency(cb.to_records()).to_records().get_column_series('latency')
            if cb_latencies:
                np.save(
                    f'{self.output_dir}/{cb.callback_name.replace("/", "__").lstrip("__")}.npy',
                    np.array(cb_latencies)
                )


class ForSingle(Common):

    @staticmethod
    def get_node_name_from_dir_name(dir_name: str) -> str:
        name_space_and_node_name = '_'.join(dir_name.split('_')[3:])
        return '/' + name_space_and_node_name.replace('__', '/')

    def get_node_from_trace_data(self, node_name: str, trace_data: str) -> Node:
        lttng = Lttng(trace_data, force_conversion=True, event_filters=[
            LttngEventFilter.duration_filter(self.cropped_duration, self.cropped_offset)])
        arch = Architecture('lttng', trace_data)
        app = Application(arch, lttng)
        return app.get_node(node_name)

    def get_nodes(self, trace_data_dir: str) -> list[Node]:
        nodes = []
        for trace_data in os.listdir(trace_data_dir):
            node_name = self.get_node_name_from_dir_name(trace_data)
            if not self.is_lsim_target(node_name):
                continue

            nodes.append(self.get_node_from_trace_data(
                node_name, f'{trace_data_dir}/{trace_data}'))

        return sorted(nodes, key=lambda node: node.node_name)

    def create(self, trace_data_dir: str) -> None:
        nodes = self.get_nodes(trace_data_dir)
        for node in nodes:
            self.create_cbs_cache(node)


class ForOriginal(Common):

    def create(self, trace_data: str) -> None:
        lttng = Lttng(trace_data, force_conversion=True, event_filters=[
            LttngEventFilter.duration_filter(self.cropped_duration, self.cropped_offset)])
        arch = Architecture('lttng', trace_data)
        app = Application(arch, lttng)
        for node in app.nodes:
            if not self.is_lsim_target(node.node_name):
                continue

            self.create_cbs_cache(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-d', "--trace_data_dir",
        default=f"{os.environ['HOME']}/autoware_optimization_tools/caret_trace_data/0")
    parser.add_argument(
        '-t', "--trace_type", choices=['original', 'single'], default='single')
    parser.add_argument('-o', "--output_dir", default='./latency_cache/single/0')
    parser.add_argument("--cropped_offset", default=25)
    parser.add_argument("--cropped_duration", default=25)
    args = parser.parse_args()

    if os.path.exists(args.output_dir):
        shutil.rmtree(args.output_dir)
    os.makedirs(args.output_dir)

    if args.trace_type == 'single':
        ForSingle(args.trace_data_dir, args.cropped_offset, args.cropped_duration, args.output_dir
                  ).create(args.trace_data_dir)
    else:
        ForOriginal(args.trace_data_dir, args.cropped_offset, args.cropped_duration,
                    args.output_dir).create(args.trace_data_dir)
