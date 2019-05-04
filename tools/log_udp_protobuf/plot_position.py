import argparse
import logging
import socketserver
import sys
import threading
import time

from google.protobuf import text_format
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
from math import degrees

from cvra_studio.viewers.LivePlotter2D import LivePlotter2D
import messages
from log_udp_protobuf import parse_packet

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--port', '-p', default=10000, help='Port to listen on (10000)')
    parser.add_argument('--verbose', '-v', action='count', default=0)
    parser.add_argument('--topic', '-t', default='/position', help='Topic name to listen for the robot position')

    return parser



def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    data_lock = threading.RLock()
    data = {
                'Start1' : {
                    'pts' : [(0,300),(0,600),(450,600),(450,300)],
                    'fill': 'r'
                },
                'Start2' : {
                    'pts' : [(0,600),(0,900),(450,900),(450,600)],
                    'fill': 'g'
                },
                'Start3' : {
                    'pts' : [(0,900),(0,1200),(450,1200),(450,900)],
                    'fill': 'b'
                },
                'Start4' : {
                    'pts' : [(2550,300),(2550,600),(3000,600),(3000,300)],
                    'fill': 'r'
                },
                'Start5' : {
                    'pts' : [(2550,600),(2550,900),(3000,900),(3000,600)],
                    'fill': 'g'
                },
                'Start6' : {
                    'pts' : [(2550,900),(2550,1200),(3000,1200),(3000,900)],
                    'fill': 'b'
                },
                'Ramp' : {
                    'pts' : [(450,2000),(450,1622),(2550,1622),(2550,2000)],
                    'fill': 'o'
                },
                'Ramp2' : {
                    'pts' : [(1228,2000),(1228,1622),(1772,1622),(1772,2000)],
                    'fill': 'o'
                },
                'MidBeacon' : {
                    'pts' : [(1300,0),(1700,0),(1700,-222),(1300,-222)],
                    'fill': 'grey'
                },
                'Experiment1' : {
                    'pts' : [(0,0),(0,-222),(450,-222),(450,0)],
                    'fill': 'grey'
                },
                'Experiment2' : {
                    'pts' : [(2550,0),(2550,-222),(3000,-222),(3000,0)],
                    'fill': 'grey'
                },
                }

    plot = LivePlotter2D((3000, 2000))
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

            if header.name != args.topic:
                return

            with data_lock:
                data.update({
                'robot': {
                    'x': msg.x,
                    'y': msg.y,
                    'a': degrees(msg.a),
                    'r': 150,
                    'n': 6,
                    'fill': 'cvra'
                },
                })

    def udp_listener():
        with socketserver.UDPServer(("0.0.0.0", args.port), Handler) as server:
            server.serve_forever()

    threading.Thread(target=udp_listener).start()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
