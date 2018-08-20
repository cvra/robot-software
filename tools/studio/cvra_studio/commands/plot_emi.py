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


import numpy as np
import scipy.optimize as so

def exponential_decay_fun(amplitude, delay, decay, constant):
    return lambda t: amplitude * np.minimum(1.0, np.exp(- (t - delay) / decay)) + constant

def exponential_decay(t, amp, delay, decay, constant):
    return exponential_decay_fun(amp, delay, decay, constant)(t)

def fit_exponential_decay(x, y, initial_guess=(1.0, 0.0005, 0.0005, 0.0)):
    return so.curve_fit(exponential_decay, x, y, initial_guess)


class EmiViewer(QtGui.QWidget):
    def __init__(self, parent = None):
        super(EmiViewer, self).__init__(parent)

        self.fit = QtGui.QLabel(parent=self)
        self.plot = LivePlotter(buffer_size=500)
        self.setLayout(vstack([self.fit, self.plot.widget]))
        self.setWindowTitle("EMI Plotter")
        self.show()

class EmiFeedbackRecorder():
    data = { 'emi': { 'time': np.array([]), 'value': np.array([]) } }

    def __init__(self, node):
        self.logger = logging.getLogger('EmiFeedbackRecorder')
        self.node = node
        self.node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal, self._callback)

    def _callback(self, event):
        freq = 48000 # Hz
        nb_samples = len(event.message.samples)
        self.data['emi']['time'] = np.linspace(0, nb_samples / freq, nb_samples)
        self.data['emi']['value'] = np.array(event.message.samples)

class EmiPlotController:
    def __init__(self, node):
        self.logger = logging.getLogger('EmiPlotController')
        self.node = node
        self.model = EmiFeedbackRecorder(node)
        self.viewer = EmiViewer()
        self.curve = self.viewer.plot.getPort()

        threading.Thread(target=self.run).start()

    def fit_exponential_decay(self, time, values):
        if len(time) < 42:
            return None
        samples = -(values - 2048) / 2048
        pw, cov = fit_exponential_decay(time, samples)
        return pw[0], pw[1]*1000, pw[2]*1000, pw[3]

    def run(self):
        self.logger.info('Emi widget started')

        def sliding_window_lp(params):
            window_length = 40
            if len(sliding_window_lp.samples) > sliding_window_lp.index:
                sliding_window_lp.samples[sliding_window_lp.index] = params
            else:
                sliding_window_lp.samples.append(params)

            sliding_window_lp.index += 1
            if window_length == sliding_window_lp.index:
                sliding_window_lp.index = 0

            return np.mean(sliding_window_lp.samples, 0)
        sliding_window_lp.index = 0
        sliding_window_lp.samples = []

        while True:
            self.curve.put(self.model.data)

            params = self.fit_exponential_decay(self.model.data['emi']['time'][19:], self.model.data['emi']['value'][19:])
            if params:
                msg = 'EMI signal fit: A={:3.4f} delay={:3.4f}ms tau={:3.4f}ms c={:3.4f}'.format(*sliding_window_lp(params))
                #msg = '{:3.7f}, {:3.7f}, {:3.7f}, {:3.7f}'.format(*params)
                self.logger.info(msg)
                self.viewer.fit.setText(msg)

            time.sleep(0.03)

def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=48))

    uavcan.load_dsdl(args.dsdl)
    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    controller = EmiPlotController(node=node)

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
