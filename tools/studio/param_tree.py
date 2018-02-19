import argparse
import logging
import queue
import threading
import time
import sys

import uavcan
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget
from network.NodeStatusMonitor import NodeStatusMonitor
from network.ParameterTree import ParameterTree, extract_value
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
    parser.add_argument('--verbose', '-v', action='count', default=3)

    return parser.parse_args()

class ParameterTreeView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.window = NestedDictView()

        self.node_selector = Selector()

        layout = QVBoxLayout()
        layout.addWidget(self.node_selector)
        layout.addWidget(self.window)
        self.setLayout(layout)

    def set(self, params):
        self.window.set(params)

    def on_node_selection(self, callback):
        self.node_selector.set_callback(callback)

    def on_edit(self, callback):
        self.window.on_edit(callback)

class ParameterTreeModel(NodeStatusMonitor):
    def __init__(self, node):
        super().__init__(node)
        self.logger = logging.getLogger('ParameterTreeModel')
        self.param_lock = threading.RLock()
        self.params = NestedDict()
        self._clear_queue()
        threading.Thread(target=self.run).start()

    def run(self):
        time.sleep(1)
        self.logger.info('Parameter tree model started')
        while True:
            target_id = self.q.get(block=True)
            if target_id:
                self.logger.info('Fetching parameters of node {}'.format(target_id))
                with self.param_lock:
                    self.params = NestedDict() # clear
                    for name, value in ParameterTree(self.node, target_id):
                        self.params.set(name.split('/'), value)
                        if self.new_params_cb:
                            self.new_params_cb(self.params)

    def on_new_params(self, callback):
        self.new_params_cb = callback

    def fetch_params(self, target_id):
        # overwrite current request by new one
        if self.q.qsize() > 0: self._clear_queue()
        self.q.put(target_id, block=False)

    def _clear_queue(self):
        self.q = queue.Queue(maxsize=1)

class ParameterTreeController():
    def __init__(self, node):
        self.logger = logging.getLogger('ParameterTreeController')

        self.view = ParameterTreeView()
        self.view.on_node_selection(self._change_selected_node)
        self.view.on_edit(self._on_param_change)
        self.view.show()

        self.model = ParameterTreeModel(node)
        self.model.on_new_node(self._update_nodes)
        self.model.on_new_params(self.view.set)

    def _update_nodes(self):
        self.node_ids = list(NodeInfo(v['name'], k) for k, v in self.model.known_nodes.items() if 'name' in v.keys())
        self.view.node_selector.set_nodes(list(node.name for node in self.node_ids))

    def _change_selected_node(self, i):
        self.selected_node = self.node_ids[i]
        self.model.fetch_params(self.selected_node.id)

    def _on_param_change(self, item):
        name = self._item_to_param_name(item)
        value = item.text()
        req = uavcan.protocol.param.GetSet.Request(
            name=name,
            value=uavcan.protocol.param.Value(real_value=float(value))
        )
        self.model.node.request(req, self.selected_node.id, self._param_changed)

    def _item_to_param_name(self, item):
        row = item.row()
        keys = []
        while True:
            item = item.parent()
            if item: keys = [item.text()] + keys
            else:    break

        child = self.model.params.get(keys)
        param = sorted(child.keys())[row]
        keys.append(param)
        return '/'.join(keys)

    def _param_changed(self, event):
        if not event: raise Exception('ParameterTree Set Timeout')
        name = event.response.name
        value = extract_value(event.response.value)
        self.logger.info('Changed parameter {} to {}'.format(name, value))

def main():
    args = parse_args()
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    app.setFont(QFont('Open Sans', pointSize=20))

    node = UavcanNode(interface=args.interface, node_id=args.id)
    param = ParameterTreeController(node)

    node.spin()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
