#!/usr/bin/env python

import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QCoreApplication
from pyqtgraph import PlotWidget


class PIDParam(QWidget):
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
            hbox.addStretch(1)
            hbox.addWidget(label)
            hbox.addWidget(field)

            self.params.append(field)
            self.vbox.addLayout(hbox)

        self.setLayout(self.vbox)


class PIDTuner(QMainWindow):
    def param_changed(self, *args):
        print("Param changed!")
        print(args)

    def create_config_panels(self):
        pages = QTabWidget()

        for loop in ['Current', 'Velocity', 'Position']:
            vbox = QVBoxLayout()
            self.params[loop.lower()] = {}

            vbox.addWidget(PIDParam())
            vbox.addWidget(QCheckBox('Plot'))

            widget = QWidget()
            widget.setLayout(vbox)
            pages.addTab(widget, loop)

        return pages

    def create_step_response_panel(self):

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

        group = QGroupBox("Step response")
        group.setLayout(vbox)

        return group

    def __init__(self):
        super().__init__()
        self.params = dict()

        self.plot_widget = PlotWidget()

        vbox = QVBoxLayout()
        vbox.addWidget(self.create_config_panels())
        vbox.addStretch(1)
        vbox.addWidget(self.create_step_response_panel())

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
