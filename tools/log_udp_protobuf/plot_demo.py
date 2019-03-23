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
        'left': {'time': np.array([0]), 'value': np.array([0])},
        'right': {'time': np.array([0]), 'value': np.array([0])},
    }

    plot = LivePlotter(10)
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

            with data_lock:
                data['left']['time'] = np.append(data['left']['time'], time.clock())
                data['right']['time'] = np.append(data['right']['time'], time.clock())
                data['left']['value'] = np.append(data['left']['value'], msg.left)
                data['right']['value'] = np.append(data['right']['value'], msg.right)


    def udp_listener():
        with socketserver.UDPServer(("0.0.0.0", args.port), Handler) as server:
            server.serve_forever()

    threading.Thread(target=udp_listener).start()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
