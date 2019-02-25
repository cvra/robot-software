#!/usr/bin/env python3
"""
UAVCAN to TUN network adapter.
"""

import argparse
import os
import sys
import uavcan
import subprocess
from queue import Queue, Empty
import threading

DSDL_DIR = os.path.join(os.path.dirname(__file__), '../uavcan_data_types/cvra')

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--interface",
        "-i",
        help="CAN Interface to use (e.g. can0 or /dev/ttyUSB0",
        required=True)
    parser.add_argument(
        "--ip-address",
        "-a",
        default="10.0.0.1/24",
        help="IP address of this interface (default 10.0.0.1/24)")
    parser.add_argument("--dsdl", help="Path to DSDL directory", default=DSDL_DIR)


    return parser.parse_args()

def open_tun_interface(tap_index, ip_addr):
    # TODO: Port this from macOS to Linux.
    tap = "tap0"
    fd = os.open("/dev/" + tap, os.O_RDWR)
    subprocess.call("ifconfig {} {}".format(tap, ip_addr).split())
    return fd


def rx_thread(tun_fd, queue):
    while True:
        packet = os.read(tun_fd, 1500)
        queue.put(packet)

def node_thread(tun_fd, node, can_to_tap, tap_to_can):
    def msg_callback(event):
        msg = event.message
        can_to_tap.put(msg.data)

    node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.DataPacket, msg_callback)

    while True:
        # A timeout of 0 means only process frames that are immediately
        # available
        try:
            node.spin(timeout=0)
        except uavcan.transport.TransferError:
            print("uavcan exception, ignoring...")
            pass

        try:
            packet = tap_to_can.get(block=False)
        except Empty:
            continue

        # Checks that the packet fits in a UWB frame
        assert len(packet) < 1024

        # Finally send it over CAN
        msg = uavcan.thirdparty.cvra.uwb_beacon.DataPacket()
        msg.dst_addr = 0xffff # broadcast
        msg.data = list(packet)

        node.broadcast(msg)

def tx_thread(tun_fd, queue):
    while True:
        packet = queue.get()
        os.write(tun_fd, bytes(packet))

def main():
    args = parse_args()
    if os.getuid() != 0:
        print("must run as root.")
        sys.exit(1)

    uavcan.load_dsdl(args.dsdl)

    tun_fd = open_tun_interface(args.ip_address)
    node = uavcan.make_node(args.interface, node_id=42)

    tap_to_can = Queue()
    can_to_tap = Queue()

    print("waiting for packets, press 3x Ctrl-C to stop...")

    rx_thd = threading.Thread(target=rx_thread, args=(tun_fd, tap_to_can))
    tx_thd = threading.Thread(target=tx_thread, args=(tun_fd, can_to_tap))
    node_thd = threading.Thread(target=node_thread, args=(tun_fd, node, can_to_tap, tap_to_can))

    rx_thd.start()
    tx_thd.start()
    node_thd.start()

    node_thd.join()
    rx_thd.join()
    tx_thd.join()



if __name__ == '__main__':
    main()
