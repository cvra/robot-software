#!/usr/bin/env python3
"""
Calibrates beacons
"""

import argparse
import uavcan
import os.path
import threading
from statistics import mean

# Speed of light in decawave units
SPEED_OF_LIGHT = 299792458.0 / (128 * 499.2e6)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--distance",
        "-d",
        type=float,
        help="Distance separating the two beacons in meters.",
        required=True,
    )
    parser.add_argument(
        "--port",
        "-p",
        help="Serial port to which the adapter is connected or name of socketcan interface.",
        required=True,
    )
    parser.add_argument(
        "--dsdl", type=str, help="Custom UAVCAN DSDL path.", required=True
    )

    return parser.parse_args()


def set_antenna_delay(node, value):
    request = uavcan.protocol.param.GetSet.Request()
    request.name = "/uwb/antenna_delay"
    request.value = uavcan.protocol.param.Value(real_value=value)


def main():
    args = parse_args()
    node = uavcan.make_node(args.port, node_id=123)
    node.lock = threading.RLock()

    uavcan.load_dsdl(args.dsdl)

    samples = []

    def range_cb(event):
        samples.append(event.message.range)
        print(event.message.range)
        if len(samples) >= 100:
            bias = mean(samples) - args.distance
            print("Estimated bias: {:.3f}".format(bias))
            print(
                "Delta to add in decawave units: {}".format(
                    int((bias / 2) / SPEED_OF_LIGHT)
                )
            )

    node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.RadioRange, range_cb)

    node.spin()


if __name__ == "__main__":
    main()
