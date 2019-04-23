import argparse
import logging
import socketserver
import sys
import threading
import time
from math import sin, cos

from google.protobuf import text_format
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui

from cvra_studio.viewers.LivePlotter2D import LivePlotter2D
import messages
from log_udp_protobuf import parse_packet

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--port', '-p', default=10000, help='Port to listen on (10000)')
    parser.add_argument('--verbose', '-v', action='count', default=0)

    return parser

def fwd_kinematics(origin, links, th1, th2, th3):
    angles = [th1, th1 + th2, th1 + th2 + th3]

    p1 = (origin[0] + links[0] * sin(angles[0]), origin[1] - links[0] * cos(angles[0]))
    p2 = (p1[0] + links[1] * sin(angles[1]), p1[1] - links[1] * cos(angles[1]))
    p3 = (p2[0] + links[2] * sin(angles[2]), p2[1] - links[2] * cos(angles[2]))

    avg = lambda x, y: (0.5 * (x[0] + y[0]), 0.5 * (x[1] + y[1]))

    l1 = avg(origin, p1)
    l2 = avg(p1, p2)
    l3 = avg(p2, p3)

    return l1, l2, l3, angles

def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    data_lock = threading.RLock()
    data = dict()

    links = (137, 97, 72)
    max_len = links[0] + links[1] + links[2]
    max_x = max_len * 2
    max_y = max_len * 1.2
    plot = LivePlotter2D((max_x, max_y))
    plot.widget.show()

    curve = plot.getPort()

    def live_plot():
        while True:
            curve.put(data)
            time.sleep(0.1)

    threading.Thread(target=live_plot).start()

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            req = self.request[0]
            header, msg = parse_packet(req)

            if header.name != '/manipulator':
                return

            with data_lock:
                origin = (max_x / 2, max_y)
                l1, l2, l3, angles = fwd_kinematics(origin, links, msg.measured.q1, msg.measured.q2, msg.measured.q3)
                i1, i2, i3, inputs = fwd_kinematics(origin, links, msg.input.q1, msg.input.q2, msg.input.q3)

                data['l1'] = {'x': l1[0], 'y': l1[1], 'w': 15, 'h': links[0], 'a': angles[0], 'color': 'cvra', 'fill': 'cvra'}
                data['l2'] = {'x': l2[0], 'y': l2[1], 'w': 12, 'h': links[1], 'a': angles[1], 'color': 'cvra', 'fill': 'cvra'}
                data['l3'] = {'x': l3[0], 'y': l3[1], 'w': 10, 'h': links[2], 'a': angles[2], 'color': 'cvra', 'fill': 'cvra'}
                data['i1'] = {'x': i1[0], 'y': i1[1], 'w': 15, 'h': links[0], 'a': inputs[0], 'color': 'g', 'fill': 'g'}
                data['i2'] = {'x': i2[0], 'y': i2[1], 'w': 12, 'h': links[1], 'a': inputs[1], 'color': 'g', 'fill': 'g'}
                data['i3'] = {'x': i3[0], 'y': i3[1], 'w': 10, 'h': links[2], 'a': inputs[2], 'color': 'g', 'fill': 'g'}


    def udp_listener():
        with socketserver.UDPServer(("0.0.0.0", args.port), Handler) as server:
            server.serve_forever()

    threading.Thread(target=udp_listener).start()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
