#!/usr/bin/env python3
"""
Tool to enforce angled brackets on our internal libraries.

It takes either a single file or will operate over whole the git tree by
default.
"""

import argparse
import re
from difflib import unified_diff
import sys
import os
import subprocess
from contextlib import contextmanager

PREFIXES = [
    "arm-cortex-tools", "aversive", "arm-cortex-tools", "aversive",
    "can-bootloader", "chibios-syscalls", "cmp", "cmp_mem_access", "crc",
    "error", "fatfs", "filter", "goap", "golem", "lwip", "msgbus", "nanopb",
    "parameter", "parameter_flash_storage", "pid", "quadramp", "raft",
    "timestamp", "uavcan", "ugfx", "version"
]


@contextmanager
def cd(dir):
    prevdir = os.getcwd()
    try:
        os.chdir(dir)
        yield
    finally:
        os.chdir(prevdir)


def fixup_line(line, lib_prefixes):
    """
    Fixes eventual includes found in this line to make sure they have the
    proper angle brackets.

    >>> fixup_line('void main() {', ['aversive'])
    'void main() {'
    >>> fixup_line('#include "aversive/oa/foo.h"', ['aversive'])
    '#include <aversive/oa/foo.h>'
    >>> fixup_line('#include "aversive/oa/foo.h" /*tmp*/', ['aversive'])
    '#include <aversive/oa/foo.h> /*tmp*/'
    >>> fixup_line('#include "aversive/oa/foo.h"', ['foo', 'aversive'])
    '#include <aversive/oa/foo.h>'
    >>> fixup_line('#include "uavcan_node.h"', ['uavcan'])
    '#include "uavcan_node.h"'
    """

    for prefix in lib_prefixes:
        expr = '^#include "({prefix}/[^"]*)"'.format(prefix=prefix)
        line = re.sub(expr, "#include <\\1>", line)

    return line


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--check',
        help='Exit with an error code if there was any changes. Useful for CI',
        action='store_true')
    parser.add_argument(
        '--inplace',
        '-i',
        action='store_true',
        help='Modify the file in place instead of showing a diff')
    parser.add_argument(
        "--input",
        type=argparse.FileType('r+'),
        help='File to process (by default operate over the tree)',
        required=False)

    return parser.parse_args()


def print_diff(input, output, filename):
    fixed_name = filename + '.fixed'
    diff = unified_diff(input, output, fromfile=filename, tofile=fixed_name)
    for l in diff:
        sys.stdout.write(l)


def repo_root():
    return os.path.join(os.path.dirname(__file__), '..')


def list_repo_files():
    prevdir = os.getcwd()

    with cd(repo_root()):
        cmd = 'git ls-tree --full-tree -r HEAD'.split()
        data = subprocess.check_output(cmd)

        data = data.decode().splitlines()
        files = [d.split('\t')[1] for d in data]

        # Filter out submodules
        files = [f for f in files if os.path.isfile(f)]

    return files


def main():
    args = parse_args()

    if args.input:
        input_files = [args.input]
    else:
        input_files = list_repo_files()
        allowed_exts = ['.h', '.hpp', '.c', '.cpp']
        input_files = [
            f for f in input_files if os.path.splitext(f)[1] in allowed_exts
        ]
        input_files = [open(f, 'r+') for f in input_files]

    changes_detected = False

    for input in input_files:
        print("{}...".format(input.name))
        input_lines = list(input)
        fixed_lines = [fixup_line(l, PREFIXES) for l in input_lines]

        if args.inplace:
            input.seek(0)
            input.write("".join(fixed_lines))
        else:
            print_diff(input_lines, fixed_lines, input.name)

        # Write down for later if we had changes
        if input_lines != fixed_lines:
            changes_detected = True

    if changes_detected and args.check:
        print("Brackets do not match coding style, exiting...")
        print(
            "If you think this is an error, consider filing an issue (@antoinealb)."
        )
        sys.exit(1)


if __name__ == '__main__':
    main()
