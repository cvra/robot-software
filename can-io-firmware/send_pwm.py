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

    parser.add_argument(
        "value",
        help="Values to set the PWM channels to (4 items)",
        type=float,
        nargs=4)

    return parser.parse_args()


def send_pwm(node, dst_id, values):
    msg = uavcan.thirdparty.cvra.io.ServoPWM(node_id=dst_id, servo_pos=values)
    node.broadcast(msg, priority=uavcan.TRANSFER_PRIORITY_HIGHEST)


def main():
    args = parse_args()

    node = uavcan.make_node(args.port, node_id=42)
    uavcan.load_dsdl(DSDL_DIR)

    send_pwm(node, args.id, args.value)

    # Spin node for 1 second
    node.spin(1)
    node.close()


if __name__ == '__main__':
    main()
