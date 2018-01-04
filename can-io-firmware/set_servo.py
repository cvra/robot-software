#!/usr/bin/env python3
"""
Sends a PWM via UAVCAN
"""

import argparse
import uavcan
import os

DSDL_DIR = os.path.join(os.path.dirname(__file__), '../uavcan_data_types/cvra')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        help="SocketCAN interface (e.g. can0) or SLCAN serial port (e.g. /dev/ttyACM0)"
    )

    parser.add_argument("id", help="ID of the board to target", type=int)

    parser.add_argument("servo", help="Servo output to be controlled", type=int)
    parser.add_argument("pos", help="Desired duty cycle on the servo output", type=float)
    parser.add_argument("vel", help="Desired duty cycle rate of change", nargs='?', default=0, type=float)
    parser.add_argument("acc", help="Desired duty cycle rate of rate of change", nargs='?', default=0, type=float)

    return parser.parse_args()

def set_servo(node, dst_id, values):
    msg = uavcan.thirdparty.cvra.io.ServoPWM(node_id=dst_id, servo_pos=values['pos'], servo_vel=values['vel'], servo_acc=values['acc'])
    node.broadcast(msg, priority=uavcan.TRANSFER_PRIORITY_HIGHEST)

def servo_setpoint(servo, pos, vel, acc):
    setpoint = {
        'pos': [0, 0, 0, 0],
        'vel': [0, 0, 0, 0],
        'acc': [0, 0, 0, 0],
    }
    setpoint['pos'][servo] = pos
    setpoint['vel'][servo] = vel
    setpoint['acc'][servo] = acc
    return setpoint

def main():
    args = parse_args()

    node = uavcan.make_node(args.port, node_id=42)
    uavcan.load_dsdl(DSDL_DIR)

    set_servo(node, args.id, servo_setpoint(args.servo, args.pos, args.vel, args.acc))

    # Spin node for 1 second
    node.spin(1)
    node.close()


if __name__ == '__main__':
    main()
