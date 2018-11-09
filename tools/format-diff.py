#!/usr/bin/env python3
"""
Formats C/C++ files to the CVRA coding standard using uncrustify.
"""

import argparse
import subprocess
import os
import shlex


def repository_root():
    toplevel = subprocess.check_output("git rev-parse --show-toplevel".split())
    toplevel = toplevel.decode().rstrip()
    return toplevel


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--all",
        "-a",
        help="Format all files (by default only changed files will be touched)",
        action='store_true'
    )

    default_cfg = os.path.join(repository_root(), 'coding-style',
                               'uncrustify.cfg')
    parser.add_argument(
        "--config",
        "-c",
        help="Config file for uncrustify",
        type=argparse.FileType(),
        default=default_cfg)

    return parser.parse_args()


def get_modified_files():
    toplevel = repository_root()
    cmd = "git diff --name-only".split()
    modified_files = subprocess.check_output(cmd).decode()
    modified_files += subprocess.check_output(cmd + ['--cached']).decode()

    # Remove directories, as those are submodules
    modified_files = [
        m for m in modified_files.splitlines()
        if not os.path.isdir(os.path.join(toplevel, m))
    ]

    return modified_files

def get_all_files():
    """
    Returns a list of all files in the git repository.
    """
    cmd = "git ls-tree --full-tree --name-only -r HEAD".split()
    files = subprocess.check_output(cmd).decode().splitlines()
    return files


def get_c_cpp_source(files):
    """
    Return a list of files matching C/C++ extensions.

    >>> get_c_cpp_source(['a.txt', 'b.c', 'd.cpp', 'e.h', 'f.hpp'])
    ['b.c', 'd.cpp', 'e.h', 'f.hpp']
    """
    allowed_extensions = ['.c', '.cpp', '.h', '.hpp']
    return [f for f in files if os.path.splitext(f)[1] in allowed_extensions]


def format_source(source_file, config):
    cmd = "uncrustify -c {cfg} --replace --no-backup {f}"
    cmd = cmd.format(cfg=config, f=source_file)
    cmd = shlex.split(cmd)

    # We have to run uncrustify twice because its parser sometimes cannot do
    # single pass formatting
    for _ in range(2):
        subprocess.call(cmd)



def main():
    args = parse_args()

    if args.all:
        files = get_all_files()
    else:
        files = get_modified_files()

    files = get_c_cpp_source(files)

    for f in files:
        format_source(f, args.config.name)


if __name__ == '__main__':
    main()
