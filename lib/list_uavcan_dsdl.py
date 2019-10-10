#!/usr/bin/env python3
"""
Lists all the UAVCAN files in a format compatible with CMake (column separated
data).

It is intended to be handled as part of the build, not be used interactively.
"""

import argparse
import glob
import os.path
import re

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", help="Generate list of outputs")
    parser.add_argument("dsdl_dirs", help="Files containing dsdl definitions", nargs="+")

    return parser.parse_args()

def output_file(input_path: str, input_dir: str, output_path: str) -> str:
    """
    Converts an input path to an output DSDLC path

    >>> output_file('foo/uavcan_data_types/cvra/20001.Reboot.uavcan', 'foo/uavcan_data_types/cvra', 'dsdlc_generated')
    'dsdlc_generated/cvra/Reboot.hpp'
    """
    input_dir = os.path.join(*input_dir.split('/')[:-1])

    path = input_path.replace(input_dir, '')
    path = path.split('/', maxsplit=1)

    # Change the extension
    path[-1] = path[-1].replace('.uavcan', '.hpp')
    path[-1] = re.sub(r'[0-9]+\.', '', path[-1])

    return os.path.join(output_path, *path)


def main():
    args = parse_args()

    input_files = []
    output_files = []

    for d in args.dsdl_dirs:
        files = glob.glob(os.path.join(d, '**/**.uavcan'), recursive=True)

        input_files += files

        if args.output_dir:
            output_files += [output_file(i, d, args.output_dir) for i in files]

    if args.output_dir:
        print(output_files[0], end='')
    else:
        print(";".join(os.path.abspath(p) for p in input_files), end='')



if __name__ == '__main__':
    main()
