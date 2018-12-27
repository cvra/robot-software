from cvra_rpc.message import *
import cvra_rpc.service_call
from trajectory_publisher import *
from math import copysign
import time
import argparse
import threading

DESTINATION = ("192.168.3.20", 20001)
TARGET = ('0.0.0.0', 20042)

TOLERANCE = 0.1
SETPOINT_FACTOR = 1.5

zero_position = 0

parser = argparse.ArgumentParser()
parser.add_argument('actuator')
args = parser.parse_args()
actuator_id = args.actuator

def server_thread(server):
    server.serve_forever()
    return

def pos_callback(args):
    if args[0] == actuator_id:
        global zero_position
        zero_position = args[1]
        print(zero_position)
    return


callbacks = {'position': pos_callback}

RequestHandler = create_request_handler(callbacks)
server = socketserver.UDPServer(TARGET, RequestHandler)

pub = SimpleRPCActuatorPublisher(("192.168.3.20", 20000))

pub.update_actuator(actuator_id, TorqueSetpoint(0))
pub.publish(time.time())

t = threading.Thread(target=server_thread, args=(server,))
t.start()

cvra_rpc.service_call.call(DESTINATION, 'config_update', [{'actuator': {actuator_id: {'stream': {'motor_pos': 30}}}}])

input("Press any key to set zero...")

cvra_rpc.service_call.call(DESTINATION, 'config_update', [{'actuator': {actuator_id: {'stream': {'motor_pos': 0}}}}])
server.shutdown()


key_stroke = input("")

setpoint = zero_position

while key_stroke != 'q':
    if key_stroke == 'w':
        setpoint += 0.1
    if key_stroke == 's':
        setpoint -= 0.1
    pub.update_actuator(actuator_id, PositionSetpoint(setpoint))
    pub.publish(time.time())
    print(setpoint)
    key_stroke = input("")
