"""
Loads all protobuf messages for the project
"""

import os.path
import glob
import importlib
import sys
from google.protobuf.message import Message

msgdir = os.path.join(
    os.path.abspath(os.path.dirname(__file__)),
    "..",
    "..",
    "master-firmware",
    "build",
    "protobuf",
)
nanopbdir = os.path.join(
    os.path.abspath(os.path.dirname(__file__)),
    "..",
    "..",
    "lib",
    "nanopb",
    "nanopb",
    "generator",
    "proto",
)

modules = glob.glob(os.path.join(msgdir, "*.py"))


sys.path.insert(0, msgdir)
sys.path.insert(0, nanopbdir)

import nanopb_pb2 as nanopb

messages = []

for f in modules:
    if not os.path.isfile(f):
        continue

    if f.endswith("__init__.py"):
        continue

    name = os.path.basename(f)[:-3]

    mod = importlib.import_module(name)

    for k, v in vars(mod).items():
        if isinstance(v, type) and issubclass(v, Message):
            globals()[k] = v
            messages.append(v)


def msgid(msg):
    return msg.DESCRIPTOR.GetOptions().Extensions[nanopb.nanopb_msgopt].msgid
