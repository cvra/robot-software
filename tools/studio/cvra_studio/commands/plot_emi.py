import argparse
import logging
import sys
import threading
import time
import uavcan

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from ..network.UavcanNode import UavcanNode
from ..viewers.LivePlotter import LivePlotter
from ..viewers.helpers import vstack


def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", default=127)
    parser.add_argument('--verbose', '-v', action='count', default=0)

    return parser

class EmiViewer(QtGui.QWidget):
    def __init__(self, parent = None):
        super(EmiViewer, self).__init__(parent)

        self.plot = LivePlotter(buffer_size=500)
        self.setLayout(vstack([self.plot.widget]))
        self.setWindowTitle("EMI Plotter")
        self.show()

class EmiFeedbackRecorder():
    data = { 'emi': { 'time': np.array([0]), 'value': np.array([0]) } }

    def __init__(self, node):
        self.logger = logging.getLogger('EmiFeedbackRecorder')
        self.node = node
        self.node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal, self._callback)

    def _callback(self, event):
        self.data['emi']['time'] = np.linspace(0, len(event.message.samples)-1, len(event.message.samples))
        self.data['emi']['value'] = np.array(event.message.samples)

class EmiPlotController:
    def __init__(self, node):
        self.logger = logging.getLogger('EmiPlotController')
        self.node = node
        self.model = EmiFeedbackRecorder(node)
        self.viewer = EmiViewer()
        self.curve = self.viewer.plot.getPort()

        threading.Thread(target=self.run).start()

    def run(self):
        self.logger.info('Emi widget started')
        while True:
            self.curve.put(self.model.data)
            time.sleep(0.03)

def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    uavcan.load_dsdl(args.dsdl)
    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    controller = EmiPlotController(node=node)

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
