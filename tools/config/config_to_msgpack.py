#!/usr/bin/env python3
import msgpack
import yaml
import argparse

from binascii import hexlify

def keys_to_str(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {str(k): keys_to_str(v) for k, v in to_convert.items()}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name',
                        required=True,
                        help='symbol name of the MessagePack buffer')
    parser.add_argument('config_file', type=argparse.FileType(),
                        help='YAML file containing the config')
    parser.add_argument('output',
                        type=argparse.FileType('w'),
                        help='Name of the MessagePack encoded output file')
    args = parser.parse_args()

    config = yaml.safe_load(args.config_file)
    config = keys_to_str(config)
    binary = msgpack.packb(config, use_single_float=True)

    args.output.write('/* generated file */\n')
    args.output.write('#include <stddef.h>\n')
    args.output.write('unsigned char {:s}[] = {{'.format(args.name))
    args.output.write('0x{:02x}'.format(binary[0]))
    for b in binary[1:]:
        args.output.write(',0x{:02x}'.format(b))
    args.output.write('};\n')
    args.output.write('const size_t {:s}_size = sizeof({});\n'.format(args.name, args.name))
    args.output.flush()
    args.output.close()

if __name__ == '__main__':
    main()
