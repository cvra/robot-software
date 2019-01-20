from collections import defaultdict
from functools import reduce
import operator
import threading

from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QWidget, QTreeView, QVBoxLayout


class NestedDict(defaultdict):
    def __init__(self):
        super().__init__(NestedDict)
        self.lock = threading.Lock()

    def get(self, mapList):
        return reduce(operator.getitem, mapList, self)

    def set(self, mapList, value):
        with self.lock: self.get(mapList[:-1])[mapList[-1]] = value

    def __repr__(self):
        return str(dict(self))

    def to_dict(self):
        if self:
            return {
                key: value.to_dict() if isinstance(value, NestedDict) else value
                for key, value in self.items()
            }
        else:
            return {}

class NestedDictView(QTreeView):
    def __init__(self):
        super().__init__()

        self.model = QStandardItemModel()
        self.model.setHorizontalHeaderLabels(['Key', 'Value'])

        self.header().setSectionResizeMode(3) # QHeaderView::ResizeMode = ResizeToContents
        self.setModel(self.model)

        self.table = NestedDict()

    def on_edit(self, callback):
        self.model.itemChanged.connect(callback)

    def clear(self, parent=None):
        parent = parent or self.model.invisibleRootItem()
        parent.removeRows(0, parent.rowCount())

    def set(self, data, parent=None, parent_node=None):
        self.clear(parent)
        parent = parent or self.model.invisibleRootItem()
        parent_node = parent_node or self.table

        for k, v in sorted(data.items()):
            if type(v) is NestedDict:
                parent.appendRow([QStandardItem(k), QStandardItem('')])
                entry = parent.child(parent.rowCount() - 1)
                parent_node[k] = NestedDict()
                self.set(v, parent=entry, parent_node=parent_node[k])
            else:
                parent.appendRow([QStandardItem(k), QStandardItem(str(v))])
                parent_node[k] = parent

        self.expand(parent.index())

class NestedDictViewWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.tree = NestedDictView()
        layout = QVBoxLayout(self)
        layout.addWidget(self.tree)
        self.setLayout(layout)

    def clear(self):
        self.tree.clear()

    def set(self, data):
        self.tree.set(data)
