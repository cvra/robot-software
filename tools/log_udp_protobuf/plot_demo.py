import argparse
import logging
import socketserver
import sys
import threading
import time

from google.protobuf import text_format
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui

from cvra_studio.viewers.LivePlotter import LivePlotter
import messages
from log_udp_protobuf import parse_packet

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--port', '-p', default=10000, help='Port to listen on (10000)')
    parser.add_argument('--verbose', '-v', action='count', default=0)

    return parser



def main(args):
    logging.basicConfig(level=max(logging.CRITICAL - (10 * args.verbose), 0))

    app = QtGui.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Open Sans', pointSize=20))

    data_lock = threading.RLock()
    data = {
        'q1': {'time': np.array([]), 'value': np.array([])},
        'q2': {'time': np.array([]), 'value': np.array([])},
        'q3': {'time': np.array([]), 'value': np.array([])},
        'ref1': {'time': np.array([]), 'value': np.array([])},
        'ref2': {'time': np.array([]), 'value': np.array([])},
        'ref3': {'time': np.array([]), 'value': np.array([])},
    }

    plot = LivePlotter(300)
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
                data['q1']['time'] = np.append(data['q1']['time'], time.clock())
                data['q1']['value'] = np.append(data['q1']['value'], msg.position.q1)
                data['q2']['time'] = np.append(data['q2']['time'], time.clock())
                data['q2']['value'] = np.append(data['q2']['value'], msg.position.q2)
                data['q3']['time'] = np.append(data['q3']['time'], time.clock())
                data['q3']['value'] = np.append(data['q3']['value'], msg.position.q3)
                data['ref1']['time'] = np.append(data['ref1']['time'], time.clock())
                data['ref1']['value'] = np.append(data['ref1']['value'], msg.reference.q1)
                data['ref2']['time'] = np.append(data['ref2']['time'], time.clock())
                data['ref2']['value'] = np.append(data['ref2']['value'], msg.reference.q2)
                data['ref3']['time'] = np.append(data['ref3']['time'], time.clock())
                data['ref3']['value'] = np.append(data['ref3']['value'], msg.reference.q3)


    def udp_listener():
        with socketserver.UDPServer(("0.0.0.0", args.port), Handler) as server:
            server.serve_forever()

    threading.Thread(target=udp_listener).start()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
