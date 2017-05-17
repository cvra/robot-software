#!/usr/bin/env python3
"""
Reads a beacon signal over CAN
"""

import argparse
import uavcan
import os
from math import degrees

DSDL_DIR = os.path.join(os.path.dirname(__file__), '../uavcan_data_types/cvra')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        help="SocketCAN interface (e.g. can0) or SLCAN serial port (e.g. /dev/ttyACM0)"
    )

    return parser.parse_args()


def beacon_cb(event):
    src = event.transfer.source_node_id
    angle = event.message.start_angle
    print("{}: {:.1f}".format(src, degrees(angle)))


def main():
    args = parse_args()

    node = uavcan.make_node(args.port)
    uavcan.load_dsdl(DSDL_DIR)

    handle = node.add_handler(uavcan.thirdparty.cvra.proximity_beacon.Signal,
                              beacon_cb)

    print("Listening for messages...")

    # Spin node for 1 second
    try:
        node.spin()
    except KeyboardInterrupt:
        node.close()


if __name__ == '__main__':
    main()
