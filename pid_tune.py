#!/usr/bin/env python

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QCoreApplication, pyqtSlot, pyqtSignal
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

    def __init__(self):
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

        self.setWindowTitle('CVRA PID tuner')
        self.show()


if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = PIDTuner()
    sys.exit(app.exec_())
