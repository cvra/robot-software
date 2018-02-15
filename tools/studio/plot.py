import queue
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from viewers.LivePlotter import LivePlotter

class Model:
    def __init__(self):
        self.data = {
            'x': {
                'time': np.array([0]),
                'value': np.array([0]),
            },
            'y': {
                'time': np.array([0]),
                'value': np.array([0]),
            },
        }
        threading.Thread(target=self.run).start()

    def get_data(self):
        return self.data

    def run(self):
        while True:
            self.data['x']['time'] = np.append(self.data['x']['time'], self.data['x']['time'][-1] + 0.1)
            self.data['y']['time'] = np.append(self.data['y']['time'], self.data['y']['time'][-1] + 0.1)
            self.data['x']['value'] = np.append(self.data['x']['value'], np.random.random(size=(1,1)))
            self.data['y']['value'] = np.append(self.data['y']['value'], np.random.random(size=(1,1)))
            time.sleep(0.1)

class Controller:
    def __init__(self):
        self.viewer = LivePlotter(buffer_size=100)
        self.curve = self.viewer.getPort()
        self.model = Model()
        threading.Thread(target=self.run).start()
        self.viewer.widget.show()

    def run(self):
        while True:
            self.curve.put(self.model.get_data())
            time.sleep(1)

def main():
    import sys
    app = QtGui.QApplication(sys.argv)

    plot_controller = Controller()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == "__main__":
    main()
