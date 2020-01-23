"""
CVRA Studio
"""

import argparse
from inspect import getmembers, isclass
import sys

from cvra_studio import __version__
from cvra_studio import commands
from cvra_studio.commands import *


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--version", "-v", action="version", version=__version__)

    subparsers = parser.add_subparsers()
    parsers = {}
    for command in commands.__all__:
        command_main = getattr(commands, command)
        parsers[command] = command_main.argparser(subparsers.add_parser(command))
        parsers[command].set_defaults(func=command_main.main)

    args = parser.parse_args()
    if hasattr(args, "func"):
        args.func(args)
    else:
        parser.print_help()
