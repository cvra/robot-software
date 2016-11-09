from cvra_rpc.service_call import call
import argparse

parser = argparse.ArgumentParser(description="Reboot all the nodes connected to the UAVCAN bus.")
parser.add_argument("master", help="IP of the master board.")
parser.add_argument("--port", "-p", type=int,
                    help="SimpleRPC port to use.", default=20001)

parser.add_argument("ids", metavar='DEVICEID', nargs='*', type=int, help="Device IDs to reboot")
parser.add_argument('-a', '--all', help="Try to scan all network.", action='store_true')

args = parser.parse_args()

if args.all:
    call((args.master, args.port), "reboot_node", [255])
else:
    call((args.master, args.port), "reboot_node", args.ids)

