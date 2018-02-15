import argparse
import datetime
import sys
import threading
import time

import uavcan

from network.UavcanNode import UavcanNode
from network.NodeStatusMonitor import NodeStatusMonitor

class NodeStatusViewer:
    def __init__(self):
        self._previous_print_len = 0
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
        self._delete_previous_print()
        formatted_line = "{:5} {:20} {:20} {:20} {:10}"
        print(formatted_line.format("ID", "Name", "Status", "Health", "Uptime"))
        for node in nodes:
            name = nodes[node]['name']
            status = nodes[node]['status']
            print(formatted_line.format(node, name, self._display_status(status.mode),
                                        self._display_health(status.health),
                                        self._display_uptime(status.uptime_sec)))
        self._previous_print_len = 1 + len(nodes)

    def _delete_previous_print(self):
        """
        Delete all previously printed lines one by one
        """
        for i in range(self._previous_print_len):
            sys.stdout.write("\033[F") # delete current line

    def _display_status(self, status):
        return self.status_messages[status]

    def _display_health(self, health):
        return self.health_messages[health]

    def _display_uptime(self, uptime_sec):
        return str(datetime.timedelta(seconds=uptime_sec))

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
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", default=127)

    return parser.parse_args()

def main():
    args = parse_args()
    uavcan.load_dsdl(args.dsdl)

    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    model = NodeStatusMonitor(node)
    viewer = NodeStatusViewer()
    controller = NodeStatusController(model, viewer)

    node.spin()

if __name__ == '__main__':
    main()
