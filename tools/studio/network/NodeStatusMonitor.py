import logging

import uavcan


class NodeStatusMonitor:
    def __init__(self, node):
        self.logger = logging.getLogger('NodeStatusMonitor')
        self.known_nodes = {}
        self.node = node
        self.node.add_handler(uavcan.protocol.NodeStatus, self._node_status_callback)
        self.on_new_node = None

    def set_on_new_node_callback(self, on_new_node):
        self.on_new_node = on_new_node

    def node_id_to_name(self, node_id):
        if node_id in self.known_nodes.keys():
            if 'name' in self.known_nodes[node_id].keys():
                return self.known_nodes[node_id]['name']
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

        if self.on_new_node is not None:
            self.on_new_node()
