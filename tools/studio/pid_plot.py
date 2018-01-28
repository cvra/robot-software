import argparse
import logging
import sys
import threading
import time
import uavcan

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from network.UavcanNode import UavcanNode
from network.NodeStatusMonitor import NodeStatusMonitor
from viewers.LivePlotter import LivePlotter
from viewers.Selector import Selector

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", default=127)
    parser.add_argument('--verbose', '-v', action='count', default=0)

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

class PidPlotController:
    def __init__(self, model, viewer):
        self.logger = logging.getLogger('PidPlotController')
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
        self.logger.info('PID widget started')
        while True:
            self.curve.put(self.data)
            time.sleep(1)

    def update_selection(self):
        self.logger.debug('New node detected, updating available nodes list')
        known_nodes = self.model.known_nodes
        nodes_with_name = {k: v for k, v in known_nodes.items() if 'name' in v.keys()}
        self.nodes = list(nodes_with_name[node]['name'] for node in nodes_with_name)
        self.viewer.node_selector.set_nodes(self.nodes)

    def _current_pid_callback(self, event):
        node_id = event.transfer.source_node_id
        node_name = self.model.node_id_to_name(node_id)
        self.logger.debug('Received current PID feedback from node {} ({})'.format(node_name, node_id))
        if self.node_to_plot is not None and self.node_to_plot == node_name:
            self.data['setpoint']['time'] = np.append(self.data['setpoint']['time'], time.time())
            self.data['measured']['time'] = np.append(self.data['measured']['time'], time.time())

            self.data['setpoint']['value'] = np.append(self.data['setpoint']['value'], event.message.current_setpoint)
            self.data['measured']['value'] = np.append(self.data['measured']['value'], event.message.current)

    def _change_selected_node(self, i):
        self.logger.debug('Current index {} selection changed to {}'.format(i, self.nodes[i]))
        self.node_to_plot = self.nodes[i]
        self.data = self._empty_pid_data()
        self.logger.info('Selected node {}'.format(self.node_to_plot))

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
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

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
