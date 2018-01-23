import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import queue
import threading
import time

class QtPlotter:
    def __init__(self):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.win = pg.GraphicsWindow()
        self.ax = self.win.addPlot()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.ax.setAspectLocked(True)

    def getPort(self):
        q = queue.Queue()
        plt = self.ax.plot()

        self.ports.append((q, plt))
        return q

    def update(self):
        for q, plt in self.ports:
            try:
                data, color = q.get(block=False)
                plt.clear()
                plt.setData(
                    np.asarray(data['x']).flatten(),
                    np.asarray(data['y']).flatten(), pen=(1,1), symbol="o",
                    symbolPen=pg.mkPen({'color': color, 'width': 2}),
                    symbolSize=1
                )
            except queue.Empty:
                pass

def qt_loop():
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

class Model:
    def __init__(self):
        self.data = {'x': np.array([0]), 'y': np.array([0])}

    def get_data(self):
        self.data['x'] = np.append(self.data['x'], self.data['x'][-1] + 1)
        self.data['y'] = np.append(self.data['y'], np.random.random(size=(1,1)))
        return self.data

class Controller:
    def __init__(self):
        self.viewer = QtPlotter()
        self.curve = self.viewer.getPort()
        self.model = Model()
        threading.Thread(target=self.run).start()

    def run(self):
        while True:
            self.curve.put((self.model.get_data(), "#00FFFF"))
            time.sleep(1)

def main():
    plot_controller = Controller()
    qt_loop()

if __name__ == "__main__":
    main()
