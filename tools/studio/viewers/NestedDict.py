from collections import defaultdict
from functools import reduce
import operator

from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QWidget, QTreeView, QVBoxLayout


class NestedDict(defaultdict):
    def __init__(self):
        super().__init__(NestedDict)

    def get(self, mapList):
        return reduce(operator.getitem, mapList, self)

    def set(self, mapList, value):
        self.get(mapList[:-1])[mapList[-1]] = value

    def __repr__(self):
        return str(dict(self))

class NestedDictView(QTreeView):
    def __init__(self):
        super().__init__()

        self.model = QStandardItemModel()
        self.model.setHorizontalHeaderLabels(['Key', 'Value'])

        self.header().setDefaultSectionSize(100)
        self.setModel(self.model)

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
