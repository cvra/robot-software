import logging

import uavcan


class NodeStatusMonitor:
    def __init__(self, node):
        self.logger = logging.getLogger('NodeStatusMonitor')
        self.known_nodes = {}
        self.node = node
        self.node.add_handler(uavcan.protocol.NodeStatus, self._node_status_callback)
        self.new_node_cb = None

    def on_new_node(self, new_node_cb):
        self.new_node_cb = new_node_cb

    def node_id_to_name(self, node_id):
        if node_id in self.known_nodes.keys():
            if 'name' in self.known_nodes[node_id].keys():
                return self.known_nodes[node_id]['name']
        return None

    def name_to_node_id(self, name):
        for node_id, info in self.known_nodes.items():
            if 'name' in self.known_nodes[node_id].keys():
                if info['name'] == name:
                    return node_id
        return None

    def _node_status_callback(self, event):
        node_id = event.transfer.source_node_id
        if node_id not in self.known_nodes:
            self.known_nodes[node_id] = {}
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), node_id, self._response_callback)
        self.known_nodes[node_id]['status'] = event.message

    def _response_callback(self, event):
        if not event:
            raise RuntimeError("Remote call timeout")

        board = event.transfer.source_node_id
        name = str(event.response.name)
        self.known_nodes[board]['name'] = name
        self.logger.info('Detected new node {} ({})'.format(self.known_nodes[board]['name'], board))

        if self.new_node_cb is not None:
            self.new_node_cb()
