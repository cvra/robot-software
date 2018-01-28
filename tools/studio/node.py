import argparse
import random
import time
import uavcan

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("id", help="UAVCAN node ID", type=int)
    parser.add_argument("name", help="UAVCAN node name", type=str)
    parser.add_argument("--dsdl", "-d", help="DSDL path")

    return parser.parse_args()

def step(scale, divider=1, max_value=1, min_value=0):
    return max_value if (round(time.time() / divider)) % 2 else min_value

def main():
    args = parse_args()
    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    node_info = uavcan.protocol.GetNodeInfo.Response(name=args.name)
    node = uavcan.make_node(args.interface, node_id=args.id, node_info=node_info)

    def publish(msg):
        node.broadcast(msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)

    def publish_current():
        publish(uavcan.thirdparty.cvra.motor.feedback.CurrentPID(
            current_setpoint=step(scale=1, divider=0.5),
            current=random.uniform(0, 1)))

    def publish_velocity():
        publish(uavcan.thirdparty.cvra.motor.feedback.VelocityPID(
            velocity_setpoint=step(scale=1, divider=1),
            velocity=random.uniform(0, 1)))

    def publish_position():
        publish(uavcan.thirdparty.cvra.motor.feedback.PositionPID(
            position_setpoint=step(scale=1, divider=2),
            position=random.uniform(0, 1)))

    if args.dsdl is not None:
        handle_current = node.periodic(0.01, publish_current)
        handle_velocity = node.periodic(0.03, publish_velocity)
        handle_position = node.periodic(0.1, publish_position)

    while True:
        try:
            node.spin(1)
        except uavcan.UAVCANException as ex:
            print('Node error:', ex)

if __name__ == '__main__':
    main()

