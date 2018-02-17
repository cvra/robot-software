import argparse
from collections import defaultdict, namedtuple
from functools import reduce
import operator
import queue
import threading
import sys

import uavcan
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QWidget, QTreeView, QVBoxLayout, QApplication
from network.UavcanNode import UavcanNode


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument("id", help="UAVCAN node ID", type=int)
    parser.add_argument("target", help="Target UAVCAN node ID", type=int)
    parser.add_argument("--dsdl", "-d", help="DSDL path")

    return parser.parse_args()

def extract_value(value):
    if   hasattr(value, 'boolean_value'): return bool(value.boolean_value)
    elif hasattr(value, 'integer_value'): return int(value.integer_value)
    elif hasattr(value, 'real_value'):    return float(value.real_value)
    elif hasattr(value, 'string_value'):  return str(value.string_value)
    else:                                 return None

Parameter = namedtuple('Parameter', ['name', 'value'])

class ParameterTree:
    def __init__(self, node, node_id):
        self.node = node
        self.node_id = node_id
        self.index = 0
        self.q = queue.Queue()
        self.done = False

    def _request_next(self):
        def callback(event):
            if event:
                self.q.put(Parameter(name=str(event.response.name), value=extract_value(event.response.value)))
                if len(event.response.name) == 0:
                    self.done = True
            else:
                raise Exception('Service request has timed out!')

        self.node.request(uavcan.protocol.param.GetSet.Request(index=self.index), self.node_id, callback)
        self.index = self.index + 1

    def __iter__(self):
        return self

    def __next__(self):
        self._request_next()
        param = self.q.get(block=True)
        if self.done:
            raise StopIteration()
        else:
            return param

class NestedDict(defaultdict):
    def __init__(self):
        super().__init__(NestedDict)

    def get(self, mapList):
        return reduce(operator.getitem, mapList, self)

    def set(self, mapList, value):
        self.get(mapList[:-1])[mapList[-1]] = value

    def __repr__(self):
        return str(dict(self))

class NestedDictView(QWidget):
    def __init__(self):
        super().__init__()
        self.tree = QTreeView(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.tree)

        self.model = QStandardItemModel()
        self.model.setHorizontalHeaderLabels(['Key', 'Value'])

        self.tree.header().setDefaultSectionSize(100)
        self.tree.setModel(self.model)

    def clear(self, parent=None):
        parent = parent or self.model.invisibleRootItem()
        parent.removeRows(0, parent.rowCount())

    def set(self, data, parent=None):
        self.clear(parent)
        parent = parent or self.model.invisibleRootItem()

        for k, v in sorted(data.items()):
            if type(v) is NestedDict:
                parent.appendRow([QStandardItem(k), QStandardItem('')])
                entry = parent.child(parent.rowCount() - 1)
                self.set(v, parent=entry)
            else:
                parent.appendRow([QStandardItem(k), QStandardItem(str(v))])

        self.tree.expand(parent.index())

def main():
    args = parse_args()
    if args.dsdl is not None:
        uavcan.load_dsdl(args.dsdl)

    app = QApplication(sys.argv)
    node = UavcanNode(interface=args.interface, node_id=args.id)

    params = NestedDict()
    window = NestedDictView()

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
