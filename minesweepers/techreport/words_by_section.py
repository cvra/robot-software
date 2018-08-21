#!/usr/bin/env python3
"""
Counts the word by sections.
"""

import argparse
import re
import collections


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", "-f", help="Tex file to read",
                        type=argparse.FileType(), default='content.tex')

    return parser.parse_args()

def main():
    args = parse_args()

    current_section = 'before'
    words = collections.OrderedDict()

    for l in args.file:
        match = re.search('\\\\section{(.*)}', l)
        if match:
            current_section = match.group(1)
        else: # content line
            words[current_section] = words.get(current_section, 0) + len(l.split())

    for section, words in words.items():
        print("{}: {} words".format(section, words))

if __name__ == '__main__':
    main()
