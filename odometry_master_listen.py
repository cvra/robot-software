#!/usr/bin/env python2
from cvra_rpc.message import *

def odometry_raw_cb(args):
    x, y, theta = tuple(args)
    print("{:.3f} {:.3f} {:.3f}".format(x, y, theta))

TARGET = ('0.0.0.0', 20000)
callbacks = {'odometry_raw': odometry_raw_cb}

RequestHandler = create_request_handler(callbacks)
server = socketserver.UDPServer(TARGET, RequestHandler)
server.serve_forever()
