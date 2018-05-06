import argparse
import logging
import sys
import time

from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QWidget

import uavcan

from ..network.UavcanNode import UavcanNode
from ..network.SetpointPublisher import ControlTopic, SetpointPublisher
from ..viewers.helpers import vstack, hstack
from ..viewers.wrappers import LineEdit, ComboBox

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", type=int, default=127)
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)

    return parser

class SetpointPublisherModel:
    def __init__(self, node, topic, motor, value, period):
        self.publisher = SetpointPublisher(node, ControlTopic(topic), motor, value, period)

    def update_motor(self, value):
        self.publisher.motor = int(value)

    def update_topic(self, value):
        self.publisher.topic = ControlTopic(value)

    def update_value(self, value):
        self.publisher.value = float(value)

    def update_period(self, value):
        self.publisher.period = float(value)

class SetpointPublisherWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.logger = logging.getLogger('SetpointPublisherWidget')

        self.model = SetpointPublisherModel(node, topic='voltage', motor=1, value=0, period=1)

        self.motor =  LineEdit(title="Motor CAN ID\t", callback=self.model.update_motor, parent=parent)
        self.topic =  ComboBox(title="Topic       \t", callback=self.model.update_topic, items=list(ControlTopic), parent=parent)
        self.value =  LineEdit(title="Value       \t", callback=self.model.update_value, parent=parent)
        self.period = LineEdit(title="Period [s]  \t", callback=self.model.update_period, parent=parent)

        self.setLayout(vstack([
            self.motor,
            self.topic,
            self.value,
            self.period,
        ]))

def main(args):
    uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    app.setFont(QFont('Open Sans', pointSize=20))

    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    pub = SetpointPublisherWidget(node)
    pub.show()

    node.spin()
    sys.exit(app.exec_())

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
