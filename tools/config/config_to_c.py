#!/usr/bin/env python3
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
    parser.add_argument('config_file', type=argparse.FileType(), help='YAML file containing the config')
    parser.add_argument('output', type=argparse.FileType('w'), help='Name of the generated C file')

    return parser.parse_args()


def main():
    args = parse_args()

    config = yaml.load(args.config_file)
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

    args.output.write(code)


if __name__ == '__main__':
    main()
