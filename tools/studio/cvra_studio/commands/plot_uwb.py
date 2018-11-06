import logging
import sys
import threading
import time
import uavcan

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from ..network.UavcanNode import UavcanNode
from ..viewers.LivePlotter2D import LivePlotter2D


def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", default=127)
    parser.add_argument('--plot_frequency', '-f', help="Plot update rate", default=30, type=float)
    parser.add_argument('--width', help="Plot width in meters", default=2, type=float)
    parser.add_argument('--height', help="Plot height in meters", default=3, type=float)
    parser.add_argument('--verbose', '-v', action='count', default=0)

    return parser


class Model:
    def __init__(self, node, size):
        self.node = node
        self.size = size
        self.node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.TagPosition, self._callback)
        self.data = {
            'robot': { 'x': 0, 'y': 0, 'a': 0, 'r': 100, 'n': 6, 'fill': 'cvra' },
            'x_cursor': { 'pts': [[0, 0], [0, self.size[1]]], 'fill': 'cvra' },
            'y_cursor': { 'pts': [[0, 0], [self.size[0], 0]], 'fill': 'cvra' },
        }

    def _callback(self, event):
        x = 1000 * event.message.x
        y = 1000 * event.message.y
        self.data.update({
            'robot': { 'x': x, 'y': y, 'a': 0, 'r': 100, 'n': 6, 'fill': 'cvra' },
            'x_cursor': { 'pts': [[x, 0], [x, self.size[1]]], 'fill': 'cvra' },
            'y_cursor': { 'pts': [[0, y], [self.size[0], y]], 'fill': 'cvra' },
        })

    def get_data(self):
        return self.data


class Controller:
    def __init__(self, node, plot_frequency, size):
        self.viewer = LivePlotter2D(size=size)
        self.plot_frequency = plot_frequency
        self.curve = self.viewer.getPort()
        self.model = Model(node, size=size)
        threading.Thread(target=self.run).start()
        self.viewer.widget.show()

    def run(self):
        while True:
            self.curve.put(self.model.get_data())
            time.sleep(1.0 / self.plot_frequency)


def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))
    app = QtGui.QApplication(sys.argv)

    uavcan.load_dsdl(args.dsdl)
    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    meter_to_mm = lambda x : x * 1000
    plot_controller = Controller(node, args.plot_frequency, (meter_to_mm(args.width), meter_to_mm(args.height)))

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == "__main__":
    args = argparser().parse_args()
    main(args)
