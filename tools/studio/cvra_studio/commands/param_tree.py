import argparse
import logging
import queue
import threading
import time
import sys
import yaml

import uavcan
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget

from ..network.NodeStatusMonitor import NodeStatusMonitor
from ..network.ParameterTree import (
    ParameterTree,
    Parameter,
    extract_value,
    value_to_uavcan,
    parameter_to_yaml,
    value_to_parameter,
)
from ..network.UavcanNode import UavcanNode
from ..viewers.NestedDict import NestedDict, NestedDictView
from ..viewers.Selector import Selector
from ..viewers.helpers import vstack, hstack

from collections import namedtuple

NodeInfo = namedtuple("NodeInfo", ["name", "id"])


def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("id", help="UAVCAN node ID", type=int)
    parser.add_argument("--dsdl", "-d", help="DSDL path")
    parser.add_argument("--verbose", "-v", action="count", default=3)

    return parser


class ParameterTreeModel(NodeStatusMonitor):
    def __init__(self, node):
        super().__init__(node)
        self.logger = logging.getLogger("ParameterTreeModel")
        self.param_lock = threading.RLock()
        self.params = NestedDict()
        self._clear_queue()
        self.target_name = ""
        threading.Thread(target=self.run).start()

    def run(self):
        time.sleep(1)
        self.logger.info("Parameter tree model started")
        while True:
            target_id = self.q.get(block=True)
            if target_id:
                self.logger.info("Fetching parameters of node {}".format(target_id))
                with self.param_lock:
                    self.params = NestedDict()  # clear
                    for parameter in ParameterTree(self.node, target_id):
                        self.params.set(parameter.name.split("/"), parameter)
                    if self.new_params_cb:
                        self.new_params_cb(self.params)
                self.logger.info(
                    "Parameters of node {} successfully fetched".format(target_id)
                )

    def on_new_params(self, callback):
        self.new_params_cb = callback

    def fetch_params(self, target_id, target_name):
        # overwrite current request by new one
        if self.q.qsize() > 0:
            self._clear_queue()
        self.q.put(target_id, block=False)
        self.target_name = target_name

    def _clear_queue(self):
        self.q = queue.Queue(maxsize=1)

    def set_param(self, target_id, name, value, value_type):
        req = uavcan.protocol.param.GetSet.Request(
            name=name, value=value_to_uavcan(value, value_type)
        )
        self.node.request(req, target_id, self._param_changed)

    def _param_changed(self, event):
        if not event:
            self.logger.warning("ParameterTree Set Timeout")
            return
        name, value = str(event.response.name), extract_value(event.response.value)[0]
        self.params.set(name.split("/"), value_to_parameter(value))
        self.logger.info(
            "Changed parameter {name} to {value}".format(name=name, value=value)
        )

    def save_params(self, target_id):
        OPCODE_SAVE = 0
        request = uavcan.protocol.param.ExecuteOpcode.Request(opcode=OPCODE_SAVE)
        self.node.request(request, target_id, self._save_params_callback)
        self.logger.info(
            "Asked node {} to save its parameters on flash".format(target_id)
        )
        self.print_params()

    def print_params(self):
        yaml.add_representer(Parameter, parameter_to_yaml)

        print("=========================YAML=========================")
        print(yaml.dump(self.params.to_dict(), default_flow_style=False))
        print("=========================YAML=========================")

    def _save_params_callback(self, event):
        if not event:
            self.logger.warning("Unable to save parameters")
        else:
            self.logger.info("Parameters saved successfully {}".format(event))

    def item_to_path(self, item):
        row = item.row()
        keys = []
        while True:
            item = item.parent()
            if item:
                keys = [item.text()] + keys
            else:
                break

        child = self.params.get(keys)
        param = sorted(child.keys())[row]
        keys.append(param)
        return keys


class ParameterWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.logger = logging.getLogger("ParameterWidget")

        self.node_selector = Selector()
        self.node_selector.set_callback(self._change_selected_node)

        self.tree_view = NestedDictView()
        self.tree_view.on_edit(self._on_param_change)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self._save_params)

        self.setLayout(vstack([self.node_selector, self.tree_view, self.save_button,]))

        self.model = ParameterTreeModel(node)
        self.model.on_new_node(self._update_nodes)
        self.model.on_new_params(self.tree_view.set)

    def _update_nodes(self):
        self.node_ids = list(
            NodeInfo(v["name"], k)
            for k, v in self.model.known_nodes.items()
            if "name" in v.keys()
        )
        self.node_selector.set_nodes(list(node.name for node in self.node_ids))

    def _change_selected_node(self, i):
        self.selected_node = self.node_ids[i]
        node = self.node_ids[self.node_selector.currentIndex()]
        self.model.fetch_params(node.id, node.name)

    def _on_param_change(self, item):
        keys = self.model.item_to_path(item)
        self.model.set_param(
            target_id=self.selected_node.id,
            name="/".join(keys),
            value=item.text(),
            value_type=self.model.params.get(keys).type,
        )

    def _save_params(self):
        self.model.save_params(self.node_ids[self.node_selector.currentIndex()].id)


def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    app.setFont(QFont("Open Sans", pointSize=20))

    node = UavcanNode(interface=args.interface, node_id=args.id)
    param = ParameterWidget(node=node)
    param.show()

    node.spin()
    sys.exit(app.exec_())


if __name__ == "__main__":
    args = argparser().parse_args()
    main(args)
