#!/usr/bin/env python

import sys
import uavcan
import threading
import argparse
import logging

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

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.logger = logging.getLogger('uavcan')
        self.board_name = {}

    def _board_info_callback(self, event):
        if not event:
            self.logger.error("Service call timed out!")
            return

        board = event.transfer.source_node_id
        name = str(event.response.name)
        logger.info('Got board info for {}'.format(board))
        self.board_name[board] = name
        self.boardDiscovered.emit(name, board)

    def node_status_callback(self, event):
        board = event.transfer.source_node_id
        if board not in self.board_name:
            self.logger.warning("Found a new board {}".format(board))
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                              self._board_info_callback)

        self.logger.info('NodeStatus from node {}'.format(board))

    def run(self):
        self.node = uavcan.make_node(self.port, node_id=127)
        self.node.add_handler(uavcan.protocol.NodeStatus,
                              self.node_status_callback)

        self.node.spin()


class PIDTuner(QMainWindow):
    @pyqtSlot(float, float, float)
    def param_changed(self, kp, ki, kd):
        print("Param changed!")

    def create_config_panels(self):
        pages = QTabWidget()

        for loop in ['Current', 'Velocity', 'Position']:
            vbox = QVBoxLayout()
            self.params[loop.lower()] = {}

            params = PIDParam()
            params.paramChanged.connect(self.param_changed)

            vbox.addWidget(params)
            vbox.addWidget(QCheckBox('Plot'))

            widget = QWidget()
            widget.setLayout(vbox)
            pages.addTab(widget, loop)

        return pages

    def __init__(self, port):
        super().__init__()
        self.params = dict()

        self.plot_widget = PlotWidget()

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
        self.can_thread.start()

        self.setWindowTitle('CVRA PID tuner')
        self.show()

        self.can_thread.boardDiscovered.connect(
            lambda name, board: QMessageBox.information(self, "Done!", "Discovered {} @ {}".format(name, board))
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        help="SocketCAN interface (e.g. can0) or SLCAN serial port (e.g. /dev/ttyACM0)"
    )

    return parser.parse_args()


if __name__ == '__main__':
    logger = logging.getLogger()

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.DEBUG)

    logger.addHandler(stream_handler)

    args = parse_args()

    app = QApplication(sys.argv)
    ex = PIDTuner(args.port)
    sys.exit(app.exec_())
