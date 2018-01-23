import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import queue

class QtPlotter:
    def __init__(self, buffer_size):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.win = pg.GraphicsWindow()
        self.ax = self.win.addPlot()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.ax.setAspectLocked(True)
        self.buffer_size = buffer_size

    def getPort(self):
        q = queue.Queue()
        plt = self.ax
        self.ports.append((q, plt))
        return q

    def update(self):
        for q, plt in self.ports:
            try:
                data = q.get(block=False)
                plt.clear()
                for index, variable in enumerate(data):
                    plt.plot(
                        np.asarray(data[variable]['time'][-self.buffer_size:]).flatten(),
                        np.asarray(data[variable]['value'][-self.buffer_size:]).flatten(),
                        pen=(index, len(data)), symbol="o",
                        symbolPen=pg.mkPen({'color': "#00FFFF", 'width': 2}),
                        symbolSize=1
                    )
            except queue.Empty:
                pass
