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

def make_cube(x, y, color, mirror):
    SIZE = 60
    RADIUS = SIZE / np.sqrt(2)
    offset = { 'y': (0, 0), 'k': (0, - SIZE), 'b': (0, SIZE), 'g': (- SIZE, 0), 'o': (SIZE, 0) }
    if mirror: offset['g'], offset['o'] = offset['o'], offset['g']
    return { 'x': x + offset[color][0], 'y': y + offset[color][1], 'a': 45, 'n': 4, 'r': RADIUS, 'fill': color }

def make_cubes(name, x, y, mirror):
    return {name.format('_y'): make_cube(x, y, 'y', mirror),
            name.format('_k'): make_cube(x, y, 'k', mirror),
            name.format('_b'): make_cube(x, y, 'b', mirror),
            name.format('_g'): make_cube(x, y, 'g', mirror),
            name.format('_o'): make_cube(x, y, 'o', mirror)}

class Model:
    def __init__(self):
        self.data = {
            'wastewater_g': { 'pts': [[894, 1750], [1500, 1750], [1500, 2000], [894, 2000]], 'fill': 'g' },
            'wastewater_o': { 'pts': [[1500, 1750], [2106, 1750], [2106, 2000], [1500, 2000]], 'fill': 'o' },
            'start_o': { 'pts': [[0, 0], [400, 0], [400, 650], [0, 650]], 'fill': 'o' },
            'start_g': { 'pts': [[3000, 0], [2600, 0], [2600, 650], [3000, 650]], 'fill': 'g' },
        }
        self.data.update(make_cubes('cube_1_o{}',  850,  540, False))
        self.data.update(make_cubes('cube_2_o{}',  300, 1190, False))
        self.data.update(make_cubes('cube_3_o{}', 1100, 1500, False))
        self.data.update(make_cubes('cube_1_g{}', 2150,  540, True))
        self.data.update(make_cubes('cube_2_g{}', 2700, 1190, True))
        self.data.update(make_cubes('cube_3_g{}', 1900, 1500, True))

        threading.Thread(target=self.run).start()

    def get_data(self):
        return self.data

    def run(self):
        while True:
            random_number = lambda: np.random.random(size=(1,1))[0][0]
            self.data.update({'robot': {
                'x': 3000 * random_number(),
                'y': 2000 * random_number(),
                'a': 2000 * random_number(),
                'r': 150,
                'n': 6,
                'fill': 'cvra'
                }})
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
