#!/usr/bin/env python2
from __future__ import division
from __future__ import absolute_import
import argparse
import logging
import sys
import threading
import time
import uavcan

from itertools import izip
from collections import deque
import numpy as np
import scipy.optimize as so

import rospy
from geometry_msgs.msg import Point
from seeker_msgs.msg import MineInfo

import ekf


def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("uavcan_dsdl_path", type=str, help="UAVCAN DSDL path")
    parser.add_argument("can_interface", type=str, help="CAN interface")
    parser.add_argument("detector_can_id", type=int, help="CAN ID of the detector to watch")
    parser.add_argument("uwb_to_detector_offset", type=float, nargs=3, help="UWB to detector offset")

    return parser


def exponential_decay_fun(amplitude, delay, decay):
    return lambda t: amplitude * np.minimum(1.0, np.exp(- (t - delay) / decay))

def exponential_decay(t, amp, delay, decay):
    return exponential_decay_fun(amp, delay, decay)(t)

def fit_exponential_decay(x, y, initial_guess=(1.0, 0.0005, 0.0005)):
    return so.curve_fit(exponential_decay, x, y, initial_guess)

def fit_signal(time, values):
    if len(time) < 42:
        return None

    samples = -(values - 2048) / 2048
    pw, cov = fit_exponential_decay(time, samples)
    return [pw[0], pw[1] * 1e3, pw[2] * 1e3], cov


def calculate_ntc_resistance(adc_value):
    r2 = 101.8  # kOhm
    if adc_value > 0:
        return r2 * (4096 - adc_value) / adc_value
    else:
        return 7000

def calculate_temperature(resistance):
    temperature_map = [[8743, -50], [4218, -40], [2132, -30], [1127, -20],
                       [620.0, -10], [353.7, 0], [208.6, 10], [126.8, 20],
                       [79.36, 30], [50.96, 40], [33.49, 50], [22.51, 60],
                       [15.44, 70], [10.80, 80], [7.686, 90], [5.556, 100],
                       [4.082, 110], [3.043, 120], [2.298, 130], [1.758, 140],
                       [1.360, 150], [1.064, 160], [0.8414, 170], [0.6714, 180],
                       [0.5408, 190], [0.4393, 200], [0.6455, 210], [0.5303, 220],
                       [0.4389, 230], [0.3658, 240], [0.5418, 250], [0.7735, 260],
                       [0.6459, 270], [0.5424, 280], [0.4583, 290], [0.3894, 300]]

    for a, b in izip(temperature_map[0:-1], temperature_map[1:]):
        if resistance < a[0] and resistance > b[0]:
            temperature = b[1] - (resistance - b[0]) / (a[0] - b[0]) * (b[1] - a[1])
            return temperature
    return None

class SlidingWindowLowPass(object):
    def __init__(self):
        self.index = 0
        self.samples = []

    def update(self, params):
        window_length = 40
        if len(self.samples) > self.index:
            self.samples[self.index] = params
        else:
            self.samples.append(params)

        self.index += 1
        if window_length == self.index:
            self.index = 0

        return np.mean(self.samples, 0)


class SpectralCentroid(object):
    def __init__(self):
        self.window_length = 32
        self.index = 0
        self.samples = np.zeros(shape=(self.window_length, 4))

    def update(self, params):
        self.samples[self.index] = params
        self.index += 1
        if self.window_length == self.index:
            self.index = 0

        spectrum = abs(np.fft.fft(self.samples)).T[0:int(self.window_length/2)+1].T
        integral = np.sum(spectrum, axis=0)
        frequency_range = np.tile(np.arange(1, self.window_length + 1), (4, 1)).T
        freq_times_integral = np.sum(frequency_range * spectrum, axis=0)

        return freq_times_integral / integral - (self.window_length + 1)/2


class EmiSignalRecorder(object):
    time = np.array([])
    signal = np.array([])
    temperature = 0.0

    def __init__(self, node, detector_id):
        self.node = node
        self.detector_id = detector_id
        self.node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal, self._callback)

    def _callback(self, event):
        if event.transfer.source_node_id != self.detector_id:
            return

        freq = 75000 # Hz
        nb_samples = len(event.message.samples) - 1
        self.time = np.linspace(0, nb_samples / freq, nb_samples)
        self.signal = np.array(event.message.samples)[0:-1]
        self.temperature = event.message.samples[-1]

class MetalMineDetector(object):
    def __init__(self, node, detector_id, uwb_to_detector_offset):
        self.node = node
        self.recorder = EmiSignalRecorder(node, detector_id)

        self.uwb_to_detector_offset = np.array(uwb_to_detector_offset)
        self.uwb_position = None
        self.last_position_update = None
        self.position_sub = rospy.Subscriber('uwb_position', Point, self._on_uwb_position_cb)

        self.detection_pub = rospy.Publisher('mine_detection', MineInfo, queue_size=1)
        self.processing_counter = 0

        self.g = lambda s, u: s
        self.G = lambda s, u: np.eye(4)
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])
        self.predictor = ekf.Predictor(self.g, self.G, self.R)

        self.h = lambda s: s
        self.H = lambda _: np.eye(4)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01])
        self.corrector = ekf.Corrector(self.h, self.H, self.Q)

        self.mu = np.zeros((4,1))
        self.sigma = 0.1 * np.eye(4)

        rospy.loginfo('Metal mine detector actively listening to node {}'.format(detector_id))

    def _on_uwb_position_cb(self, msg):
        rospy.loginfo_once('***** UWB positioning active *****')
        self.uwb_position = np.array([msg.x, msg.y, msg.z])
        self.last_position_update = rospy.Time.now()

    def run(self):
        self.sliding_window_lp = SlidingWindowLowPass()
        self.spectral_centroid = SpectralCentroid()

        last_delay = 0.
        filtered_delay = 0.
        fitted_measurements = 0

        slow_queue = deque(maxlen=20)
        fast_queue = deque(maxlen=5)

        while True:
            self.node.spin(0.1)
            temperature = calculate_temperature(calculate_ntc_resistance(self.recorder.temperature))
            params, cov = fit_signal(self.recorder.time, self.recorder.signal)
            if params:

                # Put the delay through a bandpass
                delay = params[1]
                fast_queue.append(delay)
                slow_queue.append(delay)
                fast_avg = sum(fast_queue) / len(fast_queue)
                slow_avg = sum(slow_queue) / len(slow_queue)

                filtered_delay = fast_avg - slow_avg

                if filtered_delay < -4e-3:
                    print("mine")

def main(args):
    rospy.init_node('emi_mine_detector', disable_signals=True)

    node = uavcan.make_node(args.can_interface)
    uavcan.load_dsdl(args.uavcan_dsdl_path)

    detector = MetalMineDetector(node=node, detector_id=args.detector_can_id,
                                 uwb_to_detector_offset=args.uwb_to_detector_offset)
    detector.run()

if __name__ == '__main__':
    args, _ = argparser().parse_known_args()
    main(args)
