from cvra_rpc.service_call import call
import argparse

parser = argparse.ArgumentParser(description="Reboot all the nodes connected to the UAVCAN bus.")
parser.add_argument("master", help="IP of the master board.")
parser.add_argument("--port", "-p", type=int,
                    help="SimpleRPC port to use.", default=20001)

parser.add_argument("--node", "-n", type=int, help="ID of the node to reboot (default: all).", default=255)

args = parser.parse_args()

call((args.master, args.port), "reboot_node", [args.node])

