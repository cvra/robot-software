#!/usr/bin/env python

import sys
import uavcan
import threading
import argparse
import logging
from collections import deque

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont, QDoubleValidator
from PyQt5.QtCore import QCoreApplication, pyqtSlot, pyqtSignal, QThread, QTimer
import pyqtgraph as pg


class PIDParam(QWidget):
    paramChanged = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        self.params = []
        self.vbox = QVBoxLayout()

        for param in ['Kp', 'Ki', 'Kd']:
            field = QLineEdit()
            field.setMaxLength(5)

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
        self.paramChanged.emit(1, 2, 3)


class StepConfigPanel(QGroupBox):
    parametersChanged = pyqtSignal(bool, float, float)

    def __init__(self, name):
        super().__init__(name)

        loopPicker = QComboBox()
        loopPicker.addItem("Current")
        loopPicker.addItem("Velocity")
        loopPicker.addItem("Position")
        loopPicker.currentIndexChanged.connect(self._type_changed)

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
        vbox.addWidget(loopPicker)
        vbox.addLayout(amplitude_box)
        vbox.addLayout(frequency_box)
        self.checkbox = QCheckBox('Enabled')
        vbox.addWidget(self.checkbox)

        self.setLayout(vbox)

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


class UAVCANThread(QThread):
    FREQUENCY = 10
    boardDiscovered = pyqtSignal(str, int)
    currentDataReceived = pyqtSignal(float, float, float, float)
    uavcanErrored = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.logger = logging.getLogger('uavcan')
        self.board_name = {}
        self.current_setpoint = 0
        self.publish_setpoint = False

    def _current_pid_callback(self, event):
        data = event.message
        self.logger.debug("Received current PID info: {}".format(data))
        self.currentDataReceived.emit(event.transfer.ts_monotonic,
                                      data.current_setpoint, data.current,
                                      data.motor_voltage)

    def _check_error_callback(self, event):
        if not event:
            self.logger.error("UAVCAN error")
            self.uavcanErrored.emit()

    def _board_info_callback(self, event):
        self._check_error_callback(event)

        board = event.transfer.source_node_id
        name = str(event.response.name)
        self.logger.debug('Got board info for {}'.format(board))
        self.board_name[board] = name
        self.boardDiscovered.emit(name, board)

    def _publish_setpoint(self):
        if not self.publish_setpoint:
            return

        self.logger.debug('Sending setpoint {}'.format(self.current_setpoint))

        m = uavcan.thirdparty.cvra.motor.control.Torque(
            # TODO: remove hardcoded ID
            node_id=79,
            torque=self.current_setpoint)
        self.node.broadcast(m)

    def node_status_callback(self, event):
        board = event.transfer.source_node_id
        if board not in self.board_name:
            self.logger.info("Found a new board {}".format(board))
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                              self._board_info_callback)

        self.logger.debug('NodeStatus from node {}'.format(board))

    def enable_current_pid_stream(self, board_id, enabled):
        # TODO: This is not thread safe
        self.logger.info('Enabling current PID stream for {}'.format(board_id))
        req = uavcan.thirdparty.cvra.motor.config.FeedbackStream.Request()
        req.stream = req.STREAM_CURRENT_PID
        req.enabled = enabled
        req.frequency = self.FREQUENCY
        self.node.request(req, board_id, self._check_error_callback)

    def run(self):
        self.node = uavcan.make_node(self.port, node_id=127)
        self.node.add_handler(uavcan.protocol.NodeStatus,
                              self.node_status_callback)

        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.CurrentPID,
                              self._current_pid_callback)

        self.node.periodic(0.1, self._publish_setpoint)

        self.node.spin()


class PIDTuner(QMainWindow):
    @pyqtSlot(float, float, float)
    def param_changed(self, kp, ki, kd):
        print("Param changed!")

    @pyqtSlot(float, float, float, float)
    def _received_current_data(self, timestamp, setpoint, feedback, voltage):
        self.data.append((timestamp, setpoint, feedback, voltage))
        x, y = [s[0] for s in self.data], [s[2] for s in self.data]
        self.current_plot.setData(x, y)

    @pyqtSlot(int)
    def _plot_enable(self, enabled):
        self.logger.info('Setting current plot to {}'.format(enabled))
        self.can_thread.enable_current_pid_stream(self.board_id, enabled)
        self.current_plot.setVisible(enabled)
        self.logger.debug("Step response params {} {}".format(
            self.step_config.getAmplitude(), self.step_config.getFrequency()))

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
            self.can_thread.publish_setpoint = True
        else:
            self.logger.info("Step response disabled")
            self.can_thread.publish_setpoint = False
            self.step_timer.stop()

    @pyqtSlot(str, int)
    def _discovered_board(self, name, node_id):
        if name == self.board_name:
            self.board_id = node_id
            self.setWindowTitle(
                '{} ({})'.format(self.board_name, self.board_id))

    @pyqtSlot()
    def _step_timer_timeout(self):
        if self.can_thread.current_setpoint < 0:
            self.can_thread.current_setpoint = self.step_config.getAmplitude()
        else:
            self.can_thread.current_setpoint = -self.step_config.getAmplitude()

        self.logger.debug("Step timer, current setpoint was set to {}".format(
            self.can_thread.current_setpoint))

    def create_config_panels(self):
        pages = QTabWidget()

        for loop in ['Current', 'Velocity', 'Position']:
            vbox = QVBoxLayout()
            self.params[loop.lower()] = {}

            params = PIDParam()
            params.paramChanged.connect(self.param_changed)

            vbox.addWidget(params)
            self.params[loop.lower()]['enabled'] = QCheckBox('Plot')
            vbox.addWidget(self.params[loop.lower()]['enabled'])

            widget = QWidget()
            widget.setLayout(vbox)
            pages.addTab(widget, loop)

        return pages

    def __init__(self, port, board):
        super().__init__()
        self.logger = logging.getLogger('PIDTuner')
        self.params = dict()
        self.board_name = board
        self.board_id = None

        self.setWindowTitle('{} (?)'.format(self.board_name))

        self.plot_widget = pg.PlotWidget()
        self.current_plot = self.plot_widget.plot(pen=(255, 0, 0))
        self.data = deque(maxlen=30)

        vbox = QVBoxLayout()
        vbox.addWidget(self.create_config_panels())
        vbox.addStretch(1)
        self.step_config = StepConfigPanel("Step response")
        vbox.addWidget(self.step_config)

        vbox_widget = QWidget()
        vbox_widget.setLayout(vbox)

        splitter = QSplitter()
        splitter.addWidget(self.plot_widget)
        splitter.addWidget(vbox_widget)

        self.can_thread = UAVCANThread(port)

        self.step_timer = QTimer()
        self.step_timer.timeout.connect(self._step_timer_timeout)

        # Connect all signals
        self.can_thread.currentDataReceived.connect(
            self._received_current_data)
        self.can_thread.uavcanErrored.connect(self._uavcan_errored)
        self.step_config.parametersChanged.connect(
            self._step_parameters_changed)
        self.can_thread.boardDiscovered.connect(self._discovered_board)
        for param in self.params.values():
            param['enabled'].stateChanged.connect(self._plot_enable)

        self.setCentralWidget(splitter)
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
    ex = PIDTuner(args.port, args.board)
    sys.exit(app.exec_())
