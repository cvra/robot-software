import argparse
import enum
import time

import uavcan

from ..network.UavcanNode import UavcanNode
from ..network.SetpointPublisher import ControlTopic, SetpointPublisher

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--node_id", "-n", help="UAVCAN Node ID", type=int, default=127)
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument("motor", help="Target motor UAVCAN ID", type=int)
    parser.add_argument("topic", help="Control topic (e.g. torque)", type=ControlTopic, choices=list(ControlTopic))
    parser.add_argument("value", help='Control setpoint value', type=float)
    parser.add_argument("period", help='Period of the setpoint', type=float, default=1)

    return parser

def main(args):
    uavcan.load_dsdl(args.dsdl)

    node = UavcanNode(interface=args.interface, node_id=args.node_id)

    pub = SetpointPublisher(node, args.topic, args.motor, args.value, args.period)

    node.spin()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
