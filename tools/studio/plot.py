import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import queue
from threading import Thread

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
                    np.asarray(data[0, :]).flatten(),
                    np.asarray(data[1, :]).flatten(), pen=None, symbol="o",
                    symbolPen=pg.mkPen({'color': color, 'width': 2}),
                    symbolSize=1
                )
            except queue.Empty:
                pass

def qtLoop():
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


def example():
    import time
    plotter = QtPlotter()
    curve = plotter.getPort()

    def producer():
        while True:
            curve.put((np.random.random(size=(2, 10)), "#00FFFF"))
            time.sleep(1)

    p = Thread(target=producer)
    p.daemon = True
    p.start()

    qtLoop()

if __name__ == "__main__":
    example()
