#!/usr/bin/env python

import sys
import uavcan
import threading
import argparse
import logging
from collections import deque

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QCoreApplication, pyqtSlot, pyqtSignal, QThread
from pyqtgraph import PlotWidget


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
    def __init__(self, name):
        super().__init__(name)

        loopPicker = QComboBox()
        loopPicker.addItem("Current")
        loopPicker.addItem("Velocity")
        loopPicker.addItem("Position")

        amplitude_box = QHBoxLayout()
        amplitude_box.addWidget(QLabel('Amp.'))
        amplitude_box.addWidget(QLineEdit())

        frequency_box = QHBoxLayout()
        frequency_box.addWidget(QLabel('Freq.'))
        frequency_box.addWidget(QLineEdit())

        vbox = QVBoxLayout()
        vbox.addWidget(loopPicker)
        vbox.addLayout(amplitude_box)
        vbox.addLayout(frequency_box)
        vbox.addWidget(QCheckBox('Enabled'))

        self.setLayout(vbox)


class UAVCANThread(QThread):
    boardDiscovered = pyqtSignal(str, int)
    currentDataReceived = pyqtSignal(float, float, float, float)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.logger = logging.getLogger('uavcan')
        self.board_name = {}

    def _current_pid_callback(self, event):
        data = event.message
        self.logger.debug("Received current PID info: {}".format(data))
        self.currentDataReceived.emit(event.transfer.ts_monotonic,
                                      data.current_setpoint, data.current,
                                      data.motor_voltage)

    def _board_info_callback(self, event):
        if not event:
            self.logger.error("Service call timed out!")
            return

        board = event.transfer.source_node_id
        name = str(event.response.name)
        self.logger.info('Got board info for {}'.format(board))
        self.board_name[board] = name
        self.boardDiscovered.emit(name, board)

    def node_status_callback(self, event):
        board = event.transfer.source_node_id
        if board not in self.board_name:
            self.logger.warning("Found a new board {}".format(board))
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                              self._board_info_callback)

        self.logger.info('NodeStatus from node {}'.format(board))

    def enable_current_pid_stream(self, board_id, enabled):
        # TODO: This is not thread safe
        self.logger.info('Enabling current PID stream for {}'.format(board_id))
        req = uavcan.thirdparty.cvra.motor.config.FeedbackStream.Request()
        req.stream = req.STREAM_CURRENT_PID
        req.enabled = enabled
        req.frequency = 10
        self.node.request(req, board_id, lambda *args: print(args))

    def run(self):
        self.node = uavcan.make_node(self.port, node_id=127)
        self.node.add_handler(uavcan.protocol.NodeStatus,
                              self.node_status_callback)

        self.node.add_handler(uavcan.thirdparty.cvra.motor.feedback.CurrentPID,
                              self._current_pid_callback)

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

    def _discovered_board(self, name, node_id):
        if name == self.board_name:
            self.board_id = node_id
            self.setWindowTitle(
                '{} ({})'.format(self.board_name, self.board_id))

    @pyqtSlot(int)
    def _plot_enabled(self, enabled):
        self.logger.info('Setting current plot to {}'.format(enabled))
        self.can_thread.enable_current_pid_stream(self.board_id, enabled)

    def __init__(self, port, board):
        super().__init__()
        self.logger = logging.getLogger('PIDTuner')
        self.params = dict()
        self.board_name = board
        self.board_id = None

        self.plot_widget = PlotWidget()
        self.current_plot = self.plot_widget.plot()
        self.data = deque(maxlen=30)

        vbox = QVBoxLayout()
        vbox.addWidget(self.create_config_panels())
        vbox.addStretch(1)
        vbox.addWidget(StepConfigPanel("Step response"))

        vbox_widget = QWidget()
        vbox_widget.setLayout(vbox)

        splitter = QSplitter()
        splitter.addWidget(self.plot_widget)
        splitter.addWidget(vbox_widget)

        self.setCentralWidget(splitter)

        self.can_thread = UAVCANThread(port)
        self.can_thread.currentDataReceived.connect(
            self._received_current_data)
        self.can_thread.start()

        for param in self.params.values():
            param['enabled'].stateChanged.connect(self._plot_enabled)

        self.setWindowTitle('{} (?)'.format(self.board_name))
        self.show()

        self.can_thread.boardDiscovered.connect(self._discovered_board)


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
