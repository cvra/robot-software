import argparse
import threading
import sys

import uavcan
from PyQt5.QtWidgets import QApplication
from network.UavcanNode import UavcanNode
from network.ParameterTree import ParameterTree
from viewers.NestedDict import NestedDict, NestedDictViewWidget


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("id", help="UAVCAN node ID", type=int)
    parser.add_argument("target", help="Target UAVCAN node ID", type=int)
    parser.add_argument("--dsdl", "-d", help="DSDL path")

    return parser.parse_args()

def main():
    args = parse_args()
    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    node = UavcanNode(interface=args.interface, node_id=args.id)

    params = NestedDict()
    window = NestedDictViewWidget()

    def fetch_all_params():
        for name, value in ParameterTree(node, args.target):
            params.set(name.split('/'), value)
            window.set(params)
        print('Parameters loaded:')
        print(params)
    t = threading.Thread(target=fetch_all_params).start()

    window.show()

    node.spin()
    sys.exit(app.exec_())
    t.join()

if __name__ == '__main__':
    main()
