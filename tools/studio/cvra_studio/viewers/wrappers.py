from PyQt5.QtWidgets import QWidget, QLineEdit, QLabel, QComboBox

from ..viewers.helpers import vstack, hstack

class LineEdit(QWidget):
    def __init__(self, title, initial_value=0, parent=None, callback=None):
        super().__init__(parent)
        self.label = QLabel(title, parent=parent)
        self.line = QLineEdit(str(initial_value), parent=parent)
        self.line.returnPressed.connect(self._on_value_change)
        self.callback = callback
        self.setLayout(hstack([
            self.label,
            self.line,
        ]))

    def _on_value_change(self):
        if self.callback:
            self.callback(self.line.text())

class ComboBox(QWidget):
    def __init__(self, title, items=[], parent=None, callback=None):
        super().__init__(parent)
        self.label = QLabel(title, parent=parent)
        self.combo = QComboBox(parent=parent)
        for item in items:
            self.combo.addItem(str(item))
        self.combo.currentTextChanged.connect(self._on_value_change)
        self.callback = callback
        self.setLayout(hstack([
            self.label,
            self.combo,
        ]))

    def _on_value_change(self):
        if self.callback:
            self.callback(self.combo.currentText())
