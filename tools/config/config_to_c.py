#!/usr/bin/env python3
'''
Generates C code to initialize the parameter tree given a YAML file

When given multiple YAML files as input, the generated code is compared.
If the code does not match, an error is raised.
'''
import yaml
from binascii import hexlify
import argparse

from parser.parser import parse_tree


def sanitize_keys(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers.
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {k: sanitize_keys(v) for k, v in to_convert.items()}

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file', type=argparse.FileType(), nargs='+', help='YAML file(s) containing the config')
    parser.add_argument('output', type=argparse.FileType('w'), help='Name of the generated C file')

    return parser.parse_args()


def main():
    args = parse_args()

    previous_code = None
    for file in args.config_file:
        config = yaml.safe_load(file)
        config = sanitize_keys(config)
        config = {'master': config['master']}  # Only generate code for master config

        tree = parse_tree(config)
        code = ''
        code += '#include "config.h"\n'
        code += '\n'
        code += 'static ' + tree.to_struct()
        code += '\n'
        code += '\n'
        code += tree.to_init_code('config_master_init')

        if previous_code is None:
            previous_code = code
        elif code != previous_code:
            raise RuntimeError('Input YAML files do not yield the same code!')

    args.output.write(code)


if __name__ == '__main__':
    main()
