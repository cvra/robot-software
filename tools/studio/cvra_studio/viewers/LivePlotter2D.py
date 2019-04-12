import math
from operator import itemgetter
import queue

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

COLORS = {
  'y': (247, 181, 0),
  'k': (14, 14, 16),
  'b': (0, 124, 176),
  'g': (97, 153, 59),
  'o': (208, 93, 40),
  'beige': (245, 245, 220),
  'cvra': (0, 83, 135),
}

class LivePlotter2D:
    def __init__(self, size):
        self.size = size
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.widget = pg.PlotWidget()
        self.ax = self.widget.getPlotItem()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.ax.setAspectLocked(True)
        self.ax.addLegend()
        self.ax.setXRange(0, size[0])
        self.ax.setYRange(0, size[1])

    def boundingRect(self):
        return [[0, 0], [self.size[0], 0], [self.size[0], self.size[1]], [0, self.size[1]]]

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
                plt.addItem(PolygonItem(self.boundingRect(), COLORS['k'], COLORS['beige']))
                for index, variable in enumerate(data):
                    if 'pts' in data[variable].keys():
                        pts = data[variable]['pts']
                    else:
                        pts = createPoly(
                                (data[variable]['x'], data[variable]['y']),
                                data[variable].get('n', 6),
                                data[variable].get('r', 100),
                                data[variable].get('a', 0))
                    color = data[variable].get('color', 'k')
                    fill = data[variable].get('fill', None)
                    plt.addItem(PolygonItem(pts, COLORS[color], COLORS.get(fill, None)))
            except queue.Empty:
                pass
            except Exception:
                plt.clear()


def createPoly(center, n, r, s):
    return [(center[0] + r*math.cos(math.radians(t)),
             center[1] + r*math.sin(math.radians(t)))
            for t in list(map(lambda i: i*360/n + s, range(n)))]


class PolygonItem(pg.GraphicsObject):
    def __init__(self, data, color='w', fill=None):
        pg.GraphicsObject.__init__(self)
        self.data = data
        # pre-computing a QPicture object allows paint() to run much more quickly,
        # rather than re-drawing the shapes every time.
        self.picture = QtGui.QPicture()
        p = QtGui.QPainter(self.picture)
        p.setPen(pg.mkPen(color=color))
        if fill is not None: p.setBrush(pg.mkBrush(color=fill))

        self.points = []
        for item in self.data:
            point = QtCore.QPoint(item[0], item[1])
            self.points.append(point)
        p.drawPolygon(*self.points)

        p.end()

    def paint(self, p, *args):
        p.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        # boundingRect _must_ indicate the entire area that will be drawn on
        # or else we will get artifacts and possibly crashing.
        # (in this case, QPicture does all the work of computing the bouning rect for us)
        return QtCore.QRectF(self.picture.boundingRect())
