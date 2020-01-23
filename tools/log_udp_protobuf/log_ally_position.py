#!/usr/bin/env python3
"""
Listen to UDP packets used to tell the ally robot about our position.
"""
import sys
import os.path
import argparse
import socketserver
import re
from math import degrees

from google.protobuf import text_format

import messages

PORT = 3000


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)

    return parser.parse_args()


def main():
    args = parse_args()

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            data = self.request[0]
            msg = messages.AllyPosition()
            msg.ParseFromString(data)
            print(
                "({:.03f} mm;{:.03f} mm;{:.03f}°)".format(msg.x, msg.y, degrees(msg.a))
            )

    with socketserver.UDPServer(("0.0.0.0", PORT), Handler) as server:
        server.serve_forever()


if __name__ == "__main__":
    main()
