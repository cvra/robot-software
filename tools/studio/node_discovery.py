import argparse
import threading
import time
import uavcan

class UavcanNode:
    def __init__(self, interface, node_id=None):
        self.handlers = []
        if node_id is None:
            self.node = uavcan.make_node(interface)
        else:
            self.node = uavcan.make_node(interface, node_id)

    def add_handler(self, topic, callback):
        self.handlers.append(self.node.add_handler(topic, callback))

    def spin(self):
        threading.Thread(target=self._uavcan_thread).start()

    def _uavcan_thread(self):
        while True:
            try:
                self.node.spin(0.1)
                time.sleep(0.01)
            except uavcan.UAVCANException as ex:
                print('Node error:', ex)
                self._uavcan_exit()
                return

    def _uavcan_exit(self):
        for handler in self.handlers:
            handler.remove()

class NodeStatusModel:
    def __init__(self, node):
        self.known_nodes = {}
        node.add_handler(uavcan.protocol.NodeStatus, self._node_status_callback)

    def _node_status_callback(self, event):
        self.known_nodes[event.transfer.source_node_id] = event.message

class NodeStatusViewer:
    def __init__(self):
        self.status_messages = {
            uavcan.protocol.NodeStatus().MODE_OPERATIONAL: 'OPERATIONAL',
            uavcan.protocol.NodeStatus().MODE_INITIALIZATION: 'INITIALIZATION',
            uavcan.protocol.NodeStatus().MODE_MAINTENANCE: 'MAINTENANCE',
            uavcan.protocol.NodeStatus().MODE_SOFTWARE_UPDATE: 'SOFTWARE_UPDATE',
            uavcan.protocol.NodeStatus().MODE_OFFLINE: 'OFFLINE',
        }
        self.health_messages = {
            uavcan.protocol.NodeStatus().HEALTH_OK: 'OK',
            uavcan.protocol.NodeStatus().HEALTH_WARNING: 'WARNING',
            uavcan.protocol.NodeStatus().HEALTH_ERROR: 'ERROR',
            uavcan.protocol.NodeStatus().HEALTH_CRITICAL: 'CRITICAL',
        }

    def display(self, nodes):
        print('')
        print("ID \t Status \t Health")
        for node in nodes:
            print("{} \t {} \t {}".format(node, self._display_status(nodes[node].mode),
                                          self._display_health(nodes[node].health)))

    def _display_status(self, status):
        return self.status_messages[status]

    def _display_health(self, health):
        return self.health_messages[health]

class NodeStatusController:
    def __init__(self, model, viewer):
        self.model = model
        self.viewer = viewer
        threading.Thread(target=self.show_nodes).start()

    def show_nodes(self):
        while True:
            self.viewer.display(self.model.known_nodes)
            time.sleep(1)

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)

    return parser.parse_args()

def main():
    args = parse_args()
    uavcan.load_dsdl(args.dsdl)

    node = UavcanNode(interface=args.interface)

    model = NodeStatusModel(node)
    viewer = NodeStatusViewer()
    controller = NodeStatusController(model, viewer)

    node.spin()

if __name__ == '__main__':
    main()
