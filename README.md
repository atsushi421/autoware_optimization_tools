# autoware_optimization_tools

- `parse_node_info`: component containerのplugin名から、対象ノードのpackage_name, namespace, executable_name, node_name, remap情報を取得する
- `isolated_run.bash`: 隔離環境を構築しつつ、単一ノード実行する。指定したtopicの内容を保存する機能あり。
- TODO: topicの内容比較コマンド