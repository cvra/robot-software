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
    data = { 'emi': { 'time': np.array([]), 'value': np.array([]), 'temperature': 0.0} }

    def __init__(self, node):
        self.logger = logging.getLogger('EmiFeedbackRecorder')
        self.node = node
        self.node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal, self._callback)

    def _callback(self, event):
        freq = 48000 # Hz
        nb_samples = len(event.message.samples) - 1
        self.data['emi']['time'] = np.linspace(0, nb_samples / freq, nb_samples)
        self.data['emi']['value'] = np.array(event.message.samples)[0:-1]
        self.data['emi']['temperature'] = event.message.samples[-1]

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

    def calculate_ntc_resistance(self, adc_value):
        r2 = 101.8  # kOhm
        if adc_value > 0:
            return r2 * (4096 - adc_value) / adc_value
        else:
            return 7000

    def calculate_temperature(self, resistance):
        temperature_map = [[8743, -50], [4218, -40], [2132, -30], [1127, -20],
                           [620.0, -10], [353.7, 0], [208.6, 10], [126.8, 20],
                           [79.36, 30], [50.96, 40], [33.49, 50], [22.51, 60],
                           [15.44, 70], [10.80, 80], [7.686, 90], [5.556, 100],
                           [4.082, 110], [3.043, 120], [2.298, 130], [1.758, 140],
                           [1.360, 150], [1.064, 160], [0.8414, 170], [0.6714, 180],
                           [0.5408, 190], [0.4393, 200], [0.6455, 210], [0.5303, 220],
                           [0.4389, 230], [0.3658, 240], [0.5418, 250], [0.7735, 260],
                           [0.6459, 270], [0.5424, 280], [0.4583, 290], [0.3894, 300]]

        for a, b in zip(temperature_map[0:-1], temperature_map[1:]):
            if resistance < a[0] and resistance > b[0]:
                temperature = b[1] - (resistance - b[0]) / (a[0] - b[0]) * (b[1] - a[1])
                return temperature
        return None

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
            #self.curve.put(self.model.data)

            temperature = self.calculate_temperature(self.calculate_ntc_resistance(self.model.data['emi']['temperature']))
            params = self.fit_exponential_decay(self.model.data['emi']['time'][19:], self.model.data['emi']['value'][19:])
            if params and temperature:
                delta_tau = params[2] - (0.58399854 - 0.00105068 * (temperature + 273))
                lp_params = sliding_window_lp(np.append(params, (temperature, delta_tau)))
                msg = 'EMI signal fit: A={:3.4f} delay={:3.4f}ms tau={:3.4f}ms c={:3.4f} temp={:3.2f}°C delta_tau={:3.4f}ms'.format(*lp_params)
                #msg = '{:3.7f}, {:3.7f}, {:3.7f}, {:3.7f}, {:3.7f}'.format(*np.append(params, temperature))
                self.logger.info(msg)
                #self.viewer.fit.setText(msg)

            time.sleep(0.03)

def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=42))

    uavcan.load_dsdl(args.dsdl)
    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    controller = EmiPlotController(node=node)

    node.spin()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
