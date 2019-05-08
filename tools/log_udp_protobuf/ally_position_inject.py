#!/usr/bin/env python3
"""
Displays a GUI to inject SBUS messages over the given UART port.
"""

import argparse
import sys
import socket
import messages

PORT = 3000


def udp_sender(target):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def udp_send(x, y, a):
        msg = messages.AllyPosition()
        msg.x = x
        msg.y = y
        msg.a = a

        packet = msg.SerializeToString()

        sock.sendto(packet, (target, PORT))

    return udp_send


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("udp", help="UDP host.")
    parser.add_argument("x", type=float, help="X coordinate in mm")
    parser.add_argument("y", type=float, help="Y coordinate in mm")
    parser.add_argument("a", type=float, help="Heading in deg")

    return parser.parse_args()


def main():
    args = parse_args()
    send = udp_sender(args.udp)

    send(args.x, args.y, args.a)


if __name__ == '__main__':
    main()
