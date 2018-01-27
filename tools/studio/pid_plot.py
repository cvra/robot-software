import argparse
import sys
import threading
import time
import uavcan

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from network.UavcanNode import UavcanNode
from viewers.LivePlotter import LivePlotter
from viewers.Selector import Selector

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", default=127)

    return parser.parse_args()

class PidViewer(QtGui.QWidget):
    def __init__(self, parent = None):
        super(PidViewer, self).__init__(parent)

        layout = QtGui.QVBoxLayout()

        self.node_selector = Selector()
        layout.addWidget(self.node_selector)
        self.plot = LivePlotter(buffer_size=30)
        layout.addWidget(self.plot.widget)

        self.setLayout(layout)
        self.setWindowTitle("PID Plotter")
        self.show()

class NodeStatusMonitor:
    def __init__(self, node):
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

        if self.on_new_node is not None:
            self.on_new_node()

class PidPlotController:
    def __init__(self, model, viewer):
        self.model = model
        self.viewer = viewer
        self.curve = self.viewer.plot.getPort()

        self.data = self._empty_pid_data()
        self.node_to_plot = None
        self.model.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.CurrentPID, self._current_pid_callback)

        self.nodes = []
        self.model.set_on_new_node_callback(self.update_selection)

        self.viewer.node_selector.set_callback(self._change_selected_node)

        threading.Thread(target=self.run).start()

    def run(self):
        while True:
            self.curve.put(self.data)
            time.sleep(1)

    def update_selection(self):
        known_nodes = self.model.known_nodes
        nodes_with_name = {k: v for k, v in known_nodes.items() if 'name' in v.keys()}
        self.nodes = list(nodes_with_name[node]['name'] for node in nodes_with_name)
        self.viewer.node_selector.set_nodes(self.nodes)

    def _current_pid_callback(self, event):
        node_id = event.transfer.source_node_id
        if self.node_to_plot is not None and self.node_to_plot == self.model.node_id_to_name(node_id):
            self.data['setpoint']['time'] = np.append(self.data['setpoint']['time'], time.time())
            self.data['measured']['time'] = np.append(self.data['measured']['time'], time.time())

            self.data['setpoint']['value'] = np.append(self.data['setpoint']['value'], event.message.current_setpoint)
            self.data['measured']['value'] = np.append(self.data['measured']['value'], event.message.current)

    def _change_selected_node(self, i):
        print("Current index", i, "selection changed to", self.nodes[i])
        self.node_to_plot = self.nodes[i]
        self.data = self._empty_pid_data()

    def _empty_pid_data(self):
        return {
            'setpoint': {
                'time': np.array([]),
                'value': np.array([]),
            },
            'measured': {
                'time': np.array([]),
                'value': np.array([]),
            },
        }

def main():
    args = parse_args()
    uavcan.load_dsdl(args.dsdl)

    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    controller = PidPlotController(model=NodeStatusMonitor(node), viewer=PidViewer())

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    main()
