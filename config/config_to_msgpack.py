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
    parser.add_argument('config_file', type=open, help='YAML file containing the config')
    parser.add_argument('output', help='Name of the MessagePack encoded output file')
    args = parser.parse_args()

    config = yaml.load(args.config_file)
    config = keys_to_str(config)
    binary = msgpack.packb(config, use_single_float=True)

    outfile = open(args.output, 'w')
    outfile.write('/* generated file */\n')
    outfile.write('#include <stddef.h>\n')
    outfile.write('unsigned char config_msgpack[] = {')
    outfile.write('0x{:02x}'.format(binary[0]))
    for b in binary[1:]:
        outfile.write(',0x{:02x}'.format(b))
    outfile.write('};\n')
    outfile.write('const size_t config_msgpack_size = {};\n'.format(len(binary)))
    outfile.flush()
    outfile.close()

if __name__ == '__main__':
    main()