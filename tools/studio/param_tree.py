import argparse
import threading
import sys

import uavcan
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from network.NodeStatusMonitor import NodeStatusMonitor
from network.ParameterTree import ParameterTree
from network.UavcanNode import UavcanNode
from viewers.NestedDict import NestedDict, NestedDictView
from viewers.Selector import Selector

from collections import namedtuple

NodeInfo = namedtuple('NodeInfo', ['name', 'id'])

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("id", help="UAVCAN node ID", type=int)
    parser.add_argument("--dsdl", "-d", help="DSDL path")

    return parser.parse_args()

class ParameterTreeController(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.params = NestedDict()
        self.window = NestedDictView()
        self.known_nodes = {}
        self.param_lock = threading.RLock()

        self.node_selector = Selector()
        self.node_selector.set_callback(self._change_selected_node)

        layout = QVBoxLayout()
        layout.addWidget(self.node_selector)
        layout.addWidget(self.window)
        self.setLayout(layout)

        self.monitor = NodeStatusMonitor(node)
        self.monitor.set_on_new_node_callback(self._update_nodes)

    def fetch_params(self, target_id):
        t = threading.Thread(target=self._fetch_all_params, args=(target_id,)).start()

    def _fetch_all_params(self, target_id):
        with self.param_lock:
            self.params = NestedDict() # clear
            for name, value in ParameterTree(self.node, target_id):
                self.params.set(name.split('/'), value)
                self.window.set(self.params)

    def _update_nodes(self):
        self.node_ids = list(NodeInfo(v['name'], k) for k, v in self.monitor.known_nodes.items() if 'name' in v.keys())
        self.node_selector.set_nodes(list(node.name for node in self.node_ids))

    def _change_selected_node(self, i):
        selected_node = self.node_ids[i]
        self.fetch_params(selected_node.id)

def main():
    args = parse_args()
    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    node = UavcanNode(interface=args.interface, node_id=args.id)

    param = ParameterTreeController(node)
    param.fetch_params(42)
    param.show()

    node.spin()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
