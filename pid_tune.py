#!/usr/bin/env python3

import sys
import uavcan
import threading
import argparse
import logging
from collections import deque
from multiprocessing import Lock

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont, QDoubleValidator
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QThread, QTimer
import pyqtgraph as pg


class SetpointType:
    TORQUE = 0
    VELOCITY = 1
    POSITION = 2


class PIDParam(QWidget):
    paramsChanged = pyqtSignal(float, float, float, float)

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        self.params = []
        self.vbox = QVBoxLayout()

        for param in ['Kp', 'Ki', 'Kd', 'Ilimit']:
            field = QLineEdit()
            field.setMaxLength(5)
            field.setValidator(QDoubleValidator())

            label = QLabel(param)

            hbox = QHBoxLayout()
            hbox.addWidget(label)
            hbox.addWidget(field)

            self.params.append(field)
            self.vbox.addLayout(hbox)

        for param in self.params:
            param.returnPressed.connect(self._param_changed)

        self.setLayout(self.vbox)

    @pyqtSlot()
    def _param_changed(self):
        try:
            values = [float(p.text()) for p in self.params]
            self.paramsChanged.emit(*values)
        except ValueError:
            pass


class StepConfigPanel(QGroupBox):
    parametersChanged = pyqtSignal(bool, float, float)

    def __init__(self, name):
        super().__init__(name)

        self.loopPicker = QComboBox()
        self.loopPicker.addItem("Torque")
        self.loopPicker.addItem("Velocity")
        self.loopPicker.addItem("Position")

        amplitude_box = QHBoxLayout()
        amplitude_box.addWidget(QLabel('Amp.'))
        self.amplitude_field = QLineEdit('0.')
        v = QDoubleValidator()
        v.setBottom(0)
        self.amplitude_field.setValidator(v)
        amplitude_box.addWidget(self.amplitude_field)

        frequency_box = QHBoxLayout()
        frequency_box.addWidget(QLabel('Freq.'))
        self.frequency_field = QLineEdit('1')
        v = QDoubleValidator()
        v.setBottom(0)
        self.frequency_field.setValidator(v)
        frequency_box.addWidget(self.frequency_field)

        vbox = QVBoxLayout()
        vbox.addWidget(self.loopPicker)
        vbox.addLayout(amplitude_box)
        vbox.addLayout(frequency_box)
        self.checkbox = QCheckBox('Enabled')
        vbox.addWidget(self.checkbox)

        self.setLayout(vbox)

        self.loopPicker.currentIndexChanged.connect(self._type_changed)
        self.amplitude_field.returnPressed.connect(self._param_changed)
        self.frequency_field.returnPressed.connect(self._param_changed)
        self.checkbox.stateChanged.connect(self._param_changed)

    @pyqtSlot()
    def _param_changed(self):
        self.parametersChanged.emit(self.checkbox.checkState(),
                                    self.getFrequency(), self.getAmplitude())

    @pyqtSlot()
    def _type_changed(self):
        self.checkbox.setCheckState(False)

    def getAmplitude(self):
        return float(self.amplitude_field.text())

    def getFrequency(self):
        return float(self.frequency_field.text())

    def getType(self):
        # TODO check values
        return self.loopPicker.currentIndex()


class UAVCANThread(QThread):
    FREQUENCY = 10
    boardDiscovered = pyqtSignal(str, int)
    currentDataReceived = pyqtSignal(float, float, float, float)
    velocityDataReceived = pyqtSignal(float, float, float)
    positionDataReceived = pyqtSignal(float, float, float)
    uavcanErrored = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.logger = logging.getLogger('uavcan')
        self._board_names = {}
        self.node_statuses = {}
        self.setpoint = 0
        self.setpoint_type = None
        self.setpoint_board_id = None
        self.lock = Lock()

    def _current_pid_callback(self, event):
        data = event.message
        self.logger.debug("Received current PID info: {}".format(data))
        self.currentDataReceived.emit(event.transfer.ts_monotonic,
                                      data.current_setpoint, data.current,
                                      data.motor_voltage)

    def _velocity_pid_callback(self, event):
        data = event.message
        self.logger.debug("Received velocity PID info: {}".format(data))
        self.velocityDataReceived.emit(event.transfer.ts_monotonic,
                                       data.velocity_setpoint, data.velocity)

    def _position_pid_callback(self, event):
        data = event.message
        self.logger.debug("Received position PID info: {}".format(data))
        self.positionDataReceived.emit(event.transfer.ts_monotonic,
                                       data.position_setpoint, data.position)

    def _check_error_callback(self, event):
        if not event:
            self.logger.error("UAVCAN error")
            self.uavcanErrored.emit()

    def _board_info_callback(self, event):
        self._check_error_callback(event)

        board = event.transfer.source_node_id
        name = str(event.response.name)
        self.logger.debug('Got board info for {}'.format(board))
        self._board_names[board] = name
        self.boardDiscovered.emit(name, board)

    def _publish_setpoint(self):
        if self.setpoint_board_id is None:
            return

        if self.setpoint_type == SetpointType.TORQUE:
            msg = uavcan.thirdparty.cvra.motor.control.Torque(
                node_id=self.setpoint_board_id, torque=self.setpoint)
        elif self.setpoint_type == SetpointType.VELOCITY:
            msg = uavcan.thirdparty.cvra.motor.control.Velocity(
                node_id=self.setpoint_board_id, velocity=self.setpoint)

        elif self.setpoint_type == SetpointType.POSITION:
            msg = uavcan.thirdparty.cvra.motor.control.Position(
                node_id=self.setpoint_board_id, position=self.setpoint)
        else:
            raise ValueError("Unknown setpoint type!")

        self.logger.debug('Sending setpoint {}'.format(msg))
        self.node.broadcast(msg)

    def _node_status_callback(self, event):
        board = event.transfer.source_node_id
        self.node_statuses[board] = event.message
        if board not in self._board_names:
            self.logger.info("Found a new board {}".format(board))
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                              self._board_info_callback)

        self.logger.debug('NodeStatus from node {}'.format(board))

    def enable_current_pid_stream(self, board_id, enabled):
        with self.lock:
            self.logger.info(
                'Enabling current PID stream for {}'.format(board_id))
            req = uavcan.thirdparty.cvra.motor.config.FeedbackStream.Request()
            req.stream = req.STREAM_CURRENT_PID
            req.enabled = enabled
            req.frequency = self.FREQUENCY
            self.node.request(req, board_id, self._check_error_callback)

    def enable_velocity_pid_stream(self, board_id, enabled):
        with self.lock:
            self.logger.info(
                'Enabling velocity PID stream for {}'.format(board_id))
            req = uavcan.thirdparty.cvra.motor.config.FeedbackStream.Request()
            req.stream = req.STREAM_VELOCITY_PID
            req.enabled = enabled
            req.frequency = self.FREQUENCY
            self.node.request(req, board_id, self._check_error_callback)

    def enable_position_pid_stream(self, board_id, enabled):
        with self.lock:
            self.logger.info(
                'Enabling position PID stream for {}'.format(board_id))
            req = uavcan.thirdparty.cvra.motor.config.FeedbackStream.Request()
            req.stream = req.STREAM_POSITION_PID
            req.enabled = enabled
            req.frequency = self.FREQUENCY
            self.node.request(req, board_id, self._check_error_callback)

    def set_current_gains(self, board_id, kp, ki, kd, ilim):
        with self.lock:
            self.logger.info(
                "Changing current gains to Kp={}, Ki={}, Kd={}, Ilim={}".
                format(kp, ki, kd, ilim))
            req = uavcan.thirdparty.cvra.motor.config.CurrentPID.Request()
            req.pid.kp = kp
            req.pid.ki = ki
            req.pid.kd = kd
            req.pid.ilimit = ilim
            self.node.request(req, board_id, self._check_error_callback)

    def set_velocity_gains(self, board_id, kp, ki, kd, ilim):
        with self.lock:
            self.logger.info(
                "Changing velocity gains to Kp={}, Ki={}, Kd={}, Ilim={}".
                format(kp, ki, kd, ilim))
            req = uavcan.thirdparty.cvra.motor.config.VelocityPID.Request()
            req.pid.kp = kp
            req.pid.ki = ki
            req.pid.kd = kd
            req.pid.ilimit = ilim
            self.node.request(req, board_id, self._check_error_callback)

    def set_position_gains(self, board_id, kp, ki, kd, ilim):
        with self.lock:
            self.logger.info(
                "Changing position gains to Kp={}, Ki={}, Kd={}, Ilim={}".
                format(kp, ki, kd, ilim))
            req = uavcan.thirdparty.cvra.motor.config.PositionPID.Request()
            req.pid.kp = kp
            req.pid.ki = ki
            req.pid.kd = kd
            req.pid.ilimit = ilim
            self.node.request(req, board_id, self._check_error_callback)

    def run(self):
        self.node = uavcan.make_node(self.port, node_id=127)
        self.node.add_handler(uavcan.protocol.NodeStatus,
                              self._node_status_callback)

        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.CurrentPID,
                              self._current_pid_callback)

        self.node.add_handler(
            uavcan.thirdparty.cvra.motor.feedback.VelocityPID,
            self._velocity_pid_callback)

        self.node.add_handler(
            uavcan.thirdparty.cvra.motor.feedback.PositionPID,
            self._position_pid_callback)

        self.node.periodic(0.1, self._publish_setpoint)

        while True:
            with self.lock:
                self.node.spin(0.2)


class PIDTuner(QSplitter):
    plotEnableChanged = pyqtSignal(bool)
    paramsChanged = pyqtSignal(float, float, float, float)

    @pyqtSlot(int)
    def _plot_changed(self, state):
        self.plotEnableChanged.emit(bool(state))

    @pyqtSlot(float, float, float, float)
    def _pid_changed(self, *args):
        self.paramsChanged.emit(*args)

    def __init__(self, *args):
        super().__init__(*args)
        self.plot = pg.PlotWidget()
        self.plot.addLegend()
        self.setpoint_plot = self.plot.plot(pen=(255, 0, 0), name='setpoint')
        self.feedback_plot = self.plot.plot(pen=(0, 255, 0), name='feedback')

        self.params = PIDParam()
        self.plot_enabled = QCheckBox('Plot')

        vbox = QVBoxLayout()

        vbox.addWidget(self.params)
        vbox.addWidget(self.plot_enabled)
        vbox.addStretch(1)

        box_widget = QWidget()
        box_widget.setLayout(vbox)

        self.addWidget(self.plot)
        self.addWidget(box_widget)

        self.plot_enabled.stateChanged.connect(self._plot_changed)
        self.params.paramsChanged.connect(self._pid_changed)

    def set_feedback_data(self, time, values):
        self.feedback_plot.setData(time, values)

    def set_setpoint_data(self, time, values):
        self.setpoint_plot.setData(time, values)


class PIDApp(QMainWindow):
    @pyqtSlot(float, float, float)
    def param_changed(self, kp, ki, kd):
        print("Param changed!")

    @pyqtSlot(float, float, float, float)
    def _received_current_data(self, timestamp, setpoint, feedback, voltage):
        self.current_data.append((timestamp, setpoint, feedback, voltage))

        timestamps = [s[0] for s in self.current_data]
        setpoints = [s[1] for s in self.current_data]
        feedbacks = [s[2] for s in self.current_data]

        self.current_tuner.set_setpoint_data(timestamps, setpoints)
        self.current_tuner.set_feedback_data(timestamps, feedbacks)

    @pyqtSlot(float, float, float)
    def _received_velocity_data(self, timestamp, setpoint, feedback):
        self.velocity_data.append((timestamp, setpoint, feedback))

        timestamps = [s[0] for s in self.velocity_data]
        setpoints = [s[1] for s in self.velocity_data]
        feedbacks = [s[2] for s in self.velocity_data]

        self.velocity_tuner.set_setpoint_data(timestamps, setpoints)
        self.velocity_tuner.set_feedback_data(timestamps, feedbacks)

    @pyqtSlot(float, float, float)
    def _received_position_data(self, timestamp, setpoint, feedback):
        self.position_data.append((timestamp, setpoint, feedback))

        timestamps = [s[0] for s in self.position_data]
        setpoints = [s[1] for s in self.position_data]
        feedbacks = [s[2] for s in self.position_data]

        self.position_tuner.set_setpoint_data(timestamps, setpoints)
        self.position_tuner.set_feedback_data(timestamps, feedbacks)

    @pyqtSlot(bool)
    def _current_plot_enable(self, enabled):
        self.logger.info('Setting current plot to {}'.format(enabled))
        self.can_thread.enable_current_pid_stream(self.board_id, enabled)
        self.logger.debug("Step response params {} {}".format(
            self.step_config.getAmplitude(), self.step_config.getFrequency()))

    @pyqtSlot(bool)
    def _velocity_plot_enable(self, enabled):
        self.logger.info('Setting velocity plot to {}'.format(enabled))
        self.can_thread.enable_velocity_pid_stream(self.board_id, enabled)
        self.logger.debug("Step response params {} {}".format(
            self.step_config.getAmplitude(), self.step_config.getFrequency()))

    @pyqtSlot(bool)
    def _position_plot_enable(self, enabled):
        self.logger.info('Setting position plot to {}'.format(enabled))
        self.can_thread.enable_position_pid_stream(self.board_id, enabled)
        self.logger.debug("Step response params {} {}".format(
            self.step_config.getAmplitude(), self.step_config.getFrequency()))

    @pyqtSlot(float, float, float, float)
    def _current_pid_change(self, kp, ki, kd, ilim):
        self.can_thread.set_current_gains(self.board_id, kp, ki, kd, ilim)

    @pyqtSlot(float, float, float, float)
    def _velocity_pid_change(self, kp, ki, kd, ilim):
        self.can_thread.set_velocity_gains(self.board_id, kp, ki, kd, ilim)

    @pyqtSlot(float, float, float, float)
    def _position_pid_change(self, kp, ki, kd, ilim):
        self.can_thread.set_position_gains(self.board_id, kp, ki, kd, ilim)

    @pyqtSlot()
    def _uavcan_errored(self):
        m = QMessageBox()
        m.setText("UAVCAN got an error :(")
        m.setIcon(QMessageBox.Critical)
        m.exec()

    @pyqtSlot(bool, float, float)
    def _step_parameters_changed(self, enabled, freq, amplitude):
        if enabled:
            self.logger.info("Set step response parameters: f={} Hz, amp={}".
                             format(freq, amplitude))
            self.step_timer.start(1000 / freq)
            self.can_thread.setpoint_board_id = self.board_id
            self.can_thread.setpoint_type = self.step_config.getType()
        else:
            self.logger.info("Step response disabled")
            self.can_thread.setpoint_board_id = None
            self.step_timer.stop()

    @pyqtSlot(str, int)
    def _discovered_board(self, name, node_id):
        if name == self.board_name:
            self.setEnabled(True)
            self.board_id = node_id
            self.setWindowTitle(
                '{} ({})'.format(self.board_name, self.board_id))

            # Mode 0 is MODE_OPERATIONAL
            if self.can_thread.node_statuses[self.board_id].mode != 0:
                m = QMessageBox()
                m.setText(
                    "Board not in operational mode. Some functions might not work properly."
                )
                m.setInformativeText("Did you load the initial config?")
                m.setIcon(QMessageBox.Warning)
                m.exec()

    @pyqtSlot()
    def _step_timer_timeout(self):

        if self.can_thread.setpoint < 0:
            self.can_thread.setpoint = self.step_config.getAmplitude()
        else:
            self.can_thread.setpoint = -self.step_config.getAmplitude()

        self.logger.debug("Step timer, setpoint was set to {}".format(
            self.can_thread.setpoint))

    def __init__(self, port, board):
        super().__init__()
        self.setEnabled(False)
        self.logger = logging.getLogger('PIDApp')
        self.board_name = board
        self.board_id = None

        self.setWindowTitle('{} (?)'.format(self.board_name))

        # TODO better data storage to allow zoom out
        self.current_data = deque(maxlen=30)
        self.velocity_data = deque(maxlen=30)
        self.position_data = deque(maxlen=30)

        self.current_tuner = PIDTuner()
        self.velocity_tuner = PIDTuner()
        self.position_tuner = PIDTuner()

        pages = QTabWidget()
        pages.addTab(self.current_tuner, 'Current')
        pages.addTab(self.velocity_tuner, 'Velocity')
        pages.addTab(self.position_tuner, 'Position')

        self.step_config = StepConfigPanel("Step response")

        vbox = QVBoxLayout()
        vbox.addWidget(pages)
        vbox.addWidget(self.step_config)

        vbox_widget = QWidget()
        vbox_widget.setLayout(vbox)

        self.can_thread = UAVCANThread(port)

        self.step_timer = QTimer()
        self.step_timer.timeout.connect(self._step_timer_timeout)

        # Connect all signals

        self.current_tuner.paramsChanged.connect(self._current_pid_change)
        self.velocity_tuner.paramsChanged.connect(self._velocity_pid_change)
        self.position_tuner.paramsChanged.connect(self._position_pid_change)

        self.can_thread.currentDataReceived.connect(
            self._received_current_data)
        self.current_tuner.plotEnableChanged.connect(self._current_plot_enable)
        self.can_thread.velocityDataReceived.connect(
            self._received_velocity_data)
        self.velocity_tuner.plotEnableChanged.connect(
            self._velocity_plot_enable)
        self.can_thread.positionDataReceived.connect(
            self._received_position_data)
        self.position_tuner.plotEnableChanged.connect(
            self._position_plot_enable)

        self.can_thread.uavcanErrored.connect(self._uavcan_errored)

        self.step_config.parametersChanged.connect(
            self._step_parameters_changed)
        self.can_thread.boardDiscovered.connect(self._discovered_board)

        self.setCentralWidget(vbox_widget)
        self.show()
        self.can_thread.start()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        help="SocketCAN interface (e.g. can0) or SLCAN serial port (e.g. /dev/ttyACM0)"
    )
    parser.add_argument("board", help="Board name")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument(
        "--verbose", "-v", help="Verbose mode", action='store_true')

    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    if args.verbose:
        level = logging.DEBUG
    else:
        level = logging.INFO

    logging.basicConfig(level=level)

    uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    ex = PIDApp(args.port, args.board)
    sys.exit(app.exec_())
