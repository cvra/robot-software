from operator import itemgetter
import queue

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

from .polygon_helpers import close_polygon, polygon

class LivePlotter2D:
    def __init__(self, size):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.widget = pg.PlotWidget()
        self.ax = self.widget.getPlotItem()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.ax.setAspectLocked(False)
        self.ax.addLegend()
        self.ax.setXRange(0, size[0])
        self.ax.setYRange(0, size[1])

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
                self.ax.legend.scene().removeItem(self.ax.legend)

                self.ax.addLegend()
                for index, variable in enumerate(data):
                    if 'pts' in data[variable].keys():
                        pts = data[variable]['pts']
                        x, y = list(map(itemgetter(0), pts)), list(map(itemgetter(1), pts))
                    else:
                        x, y = polygon(data[variable]['x'],
                                       data[variable]['y'],
                                       data[variable].get('r', 100),
                                       data[variable].get('n', 6),
                                       data[variable].get('a', 0))

                    x, y = close_polygon([x, y])
                    plt.plot(
                        np.asarray(x).flatten(),
                        np.asarray(y).flatten(),
                        name=variable,
                        pen=(index, len(data)),
                        symbol="o",
                        symbolSize=1
                    )
            except queue.Empty:
                pass
            except Exception:
                plt.clear()
