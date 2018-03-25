import logging
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from ..viewers.LivePlotter2D import LivePlotter2D

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verbose', '-v', action='count', default=0)

    return parser

class Model:
    def __init__(self):
        self.data = {
            'foo': {
                'x': 0,
                'y': 0,
                'a': 0,
                'r': 150,
                'n': 6,
                'a': np.pi/2,
            },
            'cube_1_y': { 'x':  850, 'y':  540, 'a': 0, 'n': 4 },
            'cube_1_g': { 'x': 2150, 'y':  540, 'a': 0, 'n': 4 },
            'cube_2_y': { 'x':  300, 'y': 1190, 'a': 0, 'n': 4 },
            'cube_2_g': { 'x': 2700, 'y': 1190, 'a': 0, 'n': 4 },
            'cube_3_y': { 'x': 1100, 'y': 1500, 'a': 0, 'n': 4 },
            'cube_3_g': { 'x': 1900, 'y': 1500, 'a': 0, 'n': 4 },
            'wastewater': { 'pts': [[894, 1750], [2106, 1750], [2106, 2000], [894, 2000]] },
            'table': { 'pts': [[0, 0], [3000, 0], [3000, 2000], [0, 2000]] },
        }
        threading.Thread(target=self.run).start()

    def get_data(self):
        return self.data

    def run(self):
        while True:
            self.data['foo']['x'] = 3000 * np.random.random(size=(1,1))[0][0]
            self.data['foo']['y'] = 2000 * np.random.random(size=(1,1))[0][0]
            time.sleep(0.1)

class Controller:
    def __init__(self):
        self.viewer = LivePlotter2D(size=(3000, 2000))
        self.curve = self.viewer.getPort()
        self.model = Model()
        threading.Thread(target=self.run).start()
        self.viewer.widget.show()

    def run(self):
        while True:
            self.curve.put(self.model.get_data())
            time.sleep(1)

def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))
    app = QtGui.QApplication(sys.argv)

    plot_controller = Controller()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == "__main__":
    args = argparser().parse_args()
    main(args)
