#!/usr/bin/env python3
"""
Read attitude information streamed by the beacon over UAVCAN. Converts them to
degrees before writing them to board.
"""

import argparse
import uavcan
import math
import csv

def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port",
        help="Serial port or SocketCAN interface")

    parser.add_argument(
        "--output",
        "-o",
        help="Log Euler angles (rad) to a CSV file",
        type=argparse.FileType('w'))

    return parser.parse_args()



def main():
    args = parse_args()
    if args.output:
        output_file = csv.DictWriter(args.output, ['ts', 'X', 'Y', 'Z'])
        output_file.writeheader()

    def orientation_cb(event):
        msg = event.message
        print(msg.timestamp.usec)
        x,y,z,w = tuple(msg.orientation_xyzw)

        x,y,z = quaternion_to_euler_angle(w,x,y,z)

        if args.output:
            output_file.writerow({'ts': msg.timestamp.usec, 'X': x, 'Y': y, 'Z': z})
        else:
            x = math.degrees(x)
            y = math.degrees(y)
            z = math.degrees(z)
            print("{:.1f}° {:.1f}° {:.1f}°".format(x, y, z))

    node = uavcan.make_node(args.port, node_id=127)
    node.add_handler(uavcan.equipment.ahrs.Solution, orientation_cb)

    node.spin()


if __name__ == '__main__':
    main()
