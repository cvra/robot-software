import argparse
import uavcan

from ..network.UavcanNode import UavcanNode

def argparser(parser=None):
    parser = parser or argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("--id", help="UAVCAN node ID", type=int, default=11)
    parser.add_argument("--name", help="UAVCAN node name", type=str, default="cmd_vel")
    parser.add_argument("--linear_x", help="Velocity in x direction [m/s]", type=float, default=0)
    parser.add_argument("--angular_z", help="Twist in yaw axis [m/s]", type=float, default=0)
    parser.add_argument("--dsdl", "-d", help="DSDL path")

    return parser

def main(args):
    if args.dsdl:
        uavcan.load_dsdl(args.dsdl)

    node = UavcanNode(interface=args.interface, node_id=args.id)

    def publish(msg):
        node.publish(msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)

    def publish_velocity_cmd():
        publish(uavcan.thirdparty.cvra.drive.VelocityCommand(
            linear_x=args.linear_x, angular_z=args.angular_z))

    handle_velocity_cmd = node.publish_periodically(0.1, publish_velocity_cmd)
    node.spin()

if __name__ == '__main__':
    args = argparser().parse_args()
    main(args)
