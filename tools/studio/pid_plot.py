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

        self.pid_loop_selector = Selector()
        layout.addWidget(self.pid_loop_selector)

        self.plot = LivePlotter(buffer_size=300)
        layout.addWidget(self.plot.widget)

        self.setLayout(layout)
        self.setWindowTitle("PID Plotter")
        self.show()

class PidFeedbackRecorder():
    nodes = []
    on_new_node_callback = None
    tracked_node = None

    PID_LOOPS = ['Current', 'Velocity', 'Position']
    tracked_pid_loop = PID_LOOPS[0]

    EMPTY_DATA = { 'setpoint': { 'time': np.array([]), 'value': np.array([]) },
                   'measured': { 'time': np.array([]), 'value': np.array([]) }, }
    data = EMPTY_DATA

    def __init__(self, node):
        self.logger = logging.getLogger('PidFeedbackRecorder')
        self.node = node
        self.data_lock = threading.RLock()

        self.monitor = NodeStatusMonitor(node)
        self.monitor.on_new_node(self._update_nodes)

        self.clear()
        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.CurrentPID, self._current_pid_callback)
        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.VelocityPID, self._velocity_pid_callback)
        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.PositionPID, self._position_pid_callback)

    def on_new_node(self, callback):
        self.on_new_node = callback

    def node_id_to_name(self, node_id):
        return self.monitor.node_id_to_name(node_id)

    def clear(self):
        with self.data_lock:
            self.data = self.EMPTY_DATA

    def _update_nodes(self):
        known_nodes = self.monitor.known_nodes
        nodes_with_name = {k: v for k, v in known_nodes.items() if 'name' in v.keys()}
        self.nodes = list(nodes_with_name[node]['name'] for node in nodes_with_name)

        if self.on_new_node:
            self.on_new_node()

    def _current_pid_callback(self, event):
        if self.tracked_pid_loop == 'Current':
            self._pid_callback(node_id=event.transfer.source_node_id,
                               setpoint=event.message.current_setpoint,
                               measured=event.message.current)

    def _velocity_pid_callback(self, event):
        if self.tracked_pid_loop == 'Velocity':
            self._pid_callback(node_id=event.transfer.source_node_id,
                               setpoint=event.message.velocity_setpoint,
                               measured=event.message.velocity)

    def _position_pid_callback(self, event):
        if self.tracked_pid_loop == 'Position':
            self._pid_callback(node_id=event.transfer.source_node_id,
                               setpoint=event.message.position_setpoint,
                               measured=event.message.position)

    def _pid_callback(self, node_id, setpoint, measured):
        node_name = self.monitor.node_id_to_name(node_id)
        if self.tracked_node is not None and self.tracked_node == node_name:
            current_time = time.time()
            self.data['setpoint']['time'] = np.append(self.data['setpoint']['time'], current_time)
            self.data['measured']['time'] = np.append(self.data['measured']['time'], current_time)
            self.data['setpoint']['value'] = np.append(self.data['setpoint']['value'], setpoint)
            self.data['measured']['value'] = np.append(self.data['measured']['value'], measured)

class PidPlotController:
    def __init__(self, node):
        self.logger = logging.getLogger('PidPlotController')
        self.node = node
        self.model = PidFeedbackRecorder(node)
        self.viewer = PidViewer()
        self.curve = self.viewer.plot.getPort()

        self.model.on_new_node(self.update_selection)

        self.viewer.node_selector.set_callback(self._change_selected_node)

        self.viewer.pid_loop_selector.set_nodes(self.model.PID_LOOPS)
        self.viewer.pid_loop_selector.set_callback(self._change_selected_pid_loop)

        threading.Thread(target=self.run).start()

    def run(self):
        self.logger.info('PID widget started')
        while True:
            self.curve.put(self.model.data)
            time.sleep(0.2)

    def update_selection(self):
        self.logger.debug('New node detected, updating available nodes list')
        self.viewer.node_selector.set_nodes(self.model.nodes)

    def _change_selected_node(self, i):
        self.model.tracked_node = self.model.nodes[i]
        self.model.clear()
        self.logger.info('Selected node {}'.format(self.model.tracked_node))

    def _change_selected_pid_loop(self, i):
        self.model.tracked_pid_loop = self.model.PID_LOOPS[i]
        self.model.clear()
        self.logger.info('Selected PID loop {}'.format(self.model.tracked_pid_loop))

def main():
    args = parse_args()
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    uavcan.load_dsdl(args.dsdl)
    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    controller = PidPlotController(node=node)

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    main()
