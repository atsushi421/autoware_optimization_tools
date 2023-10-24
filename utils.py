class NodeNameConverter:

    @staticmethod
    def to_file_name(node_name):
        return node_name.strip('/').replace('/', '_')

    @staticmethod
    def to_node_name(file_name):
        return '/' + file_name.replace('_', '/')
