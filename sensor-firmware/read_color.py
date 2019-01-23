#!/usr/bin/env python3
"""
Listens for color measurements on UAVCAN
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

    return parser.parse_args()


def color_cb(event):
    src = event.transfer.source_node_id
    r = event.message.red
    g = event.message.green
    b = event.message.blue
    c = event.message.clear
    print("{}: R {}, G {}, B {}, C {}".format(src, r, g, b, c))


def main():
    args = parse_args()

    node = uavcan.make_node(args.port)
    uavcan.load_dsdl(DSDL_DIR)

    handle = node.add_handler(uavcan.thirdparty.cvra.sensor.ColorRGBC, color_cb)

    node.spin()


if __name__ == '__main__':
    main()
