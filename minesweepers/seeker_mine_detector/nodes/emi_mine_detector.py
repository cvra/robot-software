#!/usr/bin/env python2
"""
Listens for EMI raw signals on UAVCAN, then processes it to determine if we see
a mine or not and decide to send a message it over ROS
"""

import sys

import rospy
import uavcan

import numpy as np
import scipy.optimize as so

from seeker_msgs.msg import MineInfo


def unpack_message(msg):
    freq = 75000.  # Hz
    nb_samples = len(msg.samples) - 1
    time = np.linspace(0, nb_samples / freq, nb_samples)
    value = np.array(msg.samples)[0:-1]
    temperature = msg.samples[-1]

    return time, value, temperature


def exponential_decay_fun(amplitude, delay, decay, constant):
    return lambda t: amplitude * np.minimum(1.0, np.exp(- (t - delay) / decay)) + constant


def exponential_decay(t, amp, delay, decay, constant):
    return exponential_decay_fun(amp, delay, decay, constant)(t)


def fit_exponential_decay(time, values):
    if len(time) < 42:
        return None
    samples = -(values - 2048) / 2048
    initial_guess = (1.0, 0.0005, 0.0005, 0.0)
    pw, cov = so.curve_fit(exponential_decay, time, values, initial_guess)
    return pw[0], pw[1]*1000, pw[2]*1000, pw[3]


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
                       [1.360, 150], [1.064, 160], [
                           0.8414, 170], [0.6714, 180],
                       [0.5408, 190], [0.4393, 200], [
                           0.6455, 210], [0.5303, 220],
                       [0.4389, 230], [0.3658, 240], [
                           0.5418, 250], [0.7735, 260],
                       [0.6459, 270], [0.5424, 280], [0.4583, 290], [0.3894, 300]]

    for a, b in zip(temperature_map[0:-1], temperature_map[1:]):
        if resistance < a[0] and resistance > b[0]:
            temperature = b[1] - (resistance - b[0]) / \
                (a[0] - b[0]) * (b[1] - a[1])
            return temperature

    return None


def main():
    if len(sys.argv) < 3:
        print("usage: emi_mine_detector.py uavcan_dsdl_path can_interface")
        return

    dsdl_path, can_interface = sys.argv[1], sys.argv[2]

    rospy.init_node('emi_mine_detector', disable_signals=True)
    mine_detection_pub = rospy.Publisher('metal_mine', MineInfo, queue_size=1)

    node = uavcan.make_node(can_interface)
    uavcan.load_dsdl(dsdl_path)

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

    def spectral_centroid(params):
        spectral_centroid.samples[spectral_centroid.index] = params
        spectral_centroid.index += 1
        if spectral_centroid.window_length == spectral_centroid.index:
            spectral_centroid.index = 0

        spectrum = abs(np.fft.fft(spectral_centroid.samples)
                       ).T[0:int(spectral_centroid.window_length/2)+1].T
        integral = np.sum(spectrum, axis=0)
        frequency_range = np.tile(
            np.arange(1, spectral_centroid.window_length + 1), (4, 1)).T
        freq_times_integral = np.sum(frequency_range * spectrum, axis=0)

        return freq_times_integral / integral - (spectral_centroid.window_length + 1)/2

    spectral_centroid.window_length = 32
    spectral_centroid.index = 0
    spectral_centroid.samples = np.zeros(
        shape=(spectral_centroid.window_length, 4))

    def on_emi_signal_cb(event):
        time, signal, temperature = unpack_message(event.message)

        temperature = calculate_temperature(calculate_ntc_resistance(temperature))
        params = fit_exponential_decay(time, signal)

        if params and temperature:
            lp_params = sliding_window_lp(np.append(params, (temperature)))
            spec_centroids = spectral_centroid(params)
            msg = 'EMI signal fit: A={:3.4f} delay={:3.4f}ms tau={:3.4f}ms c={:3.4f} temp={:3.2f}C'.format(*lp_params)
            rospy.logdebug(msg)

            if abs(spec_centroids[1]) > 0.2 or abs(spec_centroids[2] > 0.08):
                rospy.loginfo("Mine detected!")
                mine_detection_pub.publish(MineInfo(type=MineInfo.BURIED_LANDMINE))
            else:
                rospy.loginfo(".")

    handle = node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal,
                              on_emi_signal_cb)

    try:
        node.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
