import argparse
import enum
import time

import uavcan

from ..network.UavcanNode import UavcanNode

class ControlTopic(enum.Enum):
    voltage = 'voltage'
    torque = 'torque'
    velocity = 'velocity'
    position = 'position'

    def __str__(self):
        return self.value

    def __call__(self, node_id, value):
        return {
            'voltage': uavcan.thirdparty.cvra.motor.control.Voltage(node_id=node_id, voltage=value),
            'torque': uavcan.thirdparty.cvra.motor.control.Torque(node_id=node_id, torque=value),
            'velocity': uavcan.thirdparty.cvra.motor.control.Velocity(node_id=node_id, velocity=value),
            'position': uavcan.thirdparty.cvra.motor.control.Position(node_id=node_id, position=value),
        }[self.value]

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

    def publish():
        node.broadcast(args.topic(node_id=args.motor, value=args.value))
        time.sleep(args.period/2)
        node.broadcast(args.topic(node_id=args.motor, value=- args.value))

    node.add_publisher(args.period, publish)

    node.spin()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
