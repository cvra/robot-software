#!/usr/bin/env python3
"""
UAVCAN to TUN network adapter.
"""

import argparse
import os
import struct
import sys
import fcntl
import uavcan
import subprocess
from queue import Queue, Empty
import threading

DSDL_DIR = os.path.join(os.path.dirname(__file__), '../../uavcan_data_types/cvra')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--interface",
        "-i",
        help="CAN Interface to use (e.g. can0 or /dev/ttyUSB0",
        required=True)
    parser.add_argument("--dsdl", help="Path to DSDL directory", default=DSDL_DIR)

    return parser.parse_args()


def main():
    args = parse_args()

    uavcan.load_dsdl(args.dsdl)

    node = uavcan.make_node(args.interface, node_id=42)

    def do_publish():
        msg = uavcan.thirdparty.cvra.uwb_beacon.DataPacket()
        msg.dst_addr = 42
        node.broadcast(msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)

    handle = node.periodic(1, do_publish)
    node.spin()


if __name__ == '__main__':
    main()
