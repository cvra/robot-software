from pyqtgraph.Qt import QtGui


class Selector(QtGui.QComboBox):
    def __init__(self):
        super(Selector, self).__init__()

    def set_nodes(self, nodes):
        try:
            self.clear()
            self.addItems(nodes)
        except:
            pass
        self.update()

    def set_callback(self, callback):
        self.currentIndexChanged.connect(callback)


class SelectorWidget(QtGui.QWidget):
    def __init__(self, parent=None, title=""):
        super(SelectorWidget, self).__init__(parent)

        layout = QtGui.QHBoxLayout()

        self.node_selector = Selector()
        layout.addWidget(self.node_selector)

        self.setLayout(layout)
        self.setWindowTitle(title)

    def set_nodes(self, nodes):
        self.node_selector.set_nodes(nodes)

    def set_callback(self, callback):
        self.node_selector.set_callback(callback)
