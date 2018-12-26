from cvra_rpc.message import *
import cvra_rpc.service_call
import argparse
import sys

DESTINATION = ("192.168.3.20", 20001)

server = []

def pos_cb(args):
    print(args)
    cvra_rpc.service_call.call(DESTINATION, 'config_update', [{'actuator': {args[0]: {'stream': {'motor_pos': 0}}}}])
    global server
    server.shutdown()



TARGET = ('0.0.0.0', 20042)

parser = argparse.ArgumentParser()
parser.add_argument('actuator')
args = parser.parse_args()

callbacks = {'position': pos_cb}
cvra_rpc.service_call.call(DESTINATION, 'config_update', [{'actuator': {args.actuator: {'stream': {'motor_pos': 100}}}}])

RequestHandler = create_request_handler(callbacks)
server = socketserver.UDPServer(TARGET, RequestHandler)
server.serve_forever()
