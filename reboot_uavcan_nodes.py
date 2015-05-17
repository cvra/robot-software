from cvra_rpc.service_call import call
import argparse

parser = argparse.ArgumentParser(description="Reboot all the nodes connected to the UAVCAN bus.")
parser.add_argument("master", help="IP of the master board.")
parser.add_argument("--port", "-p", type=int,
                    help="SimpleRPC port to use.", default=20001)

args = parser.parse_args()

call((args.master, args.port), "uavcan_reboot_nodes", [])

