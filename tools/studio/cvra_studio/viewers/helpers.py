from functools import singledispatch
from PyQt5.QtGui import QBoxLayout, QVBoxLayout, QHBoxLayout, QWidget


def vstack(items):
    layout = QVBoxLayout()
    for item in items:
        stack(item, layout)
    return layout


def hstack(items):
    layout = QHBoxLayout()
    for item in items:
        stack(item, layout)
    return layout


@singledispatch
def stack(item, layout):
    raise Exception("Non valid item of type", type(item))


@stack.register(QBoxLayout)
def _(item, layout):
    layout.addLayout(item)


@stack.register(QWidget)
def _(item, layout):
    layout.addWidget(item)
