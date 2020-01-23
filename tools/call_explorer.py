#!/usr/bin/env python3
"""
Finds all the callers for a given function, given a disassembly (list) file.
This can be useful, i.e. when tracking down why a binary is getting very large.

The callee name is given as a regular expression, which is useful when the
exact matching name is not known (e.g. free vs _free_r).

Example invocation used to track down why printf gets included in a build:
$ call_explorer.py can-io-firmware/build/ch.list malloc
fiprintf      ->  _vfiprintf_r
__assert_func ->  fiprintf
_vfiprintf_r  ->  __sbprintf
__sbprintf    ->  _vfiprintf_r

Here we see that fiprintf is called by __assert_func. call_explorer.py could
then be ran again, targetting __assert_func to walk up the calling stack until
the culprit is found.
"""

import argparse
import re
import subprocess

NAME_PATTERN = "^[0-9A-Fa-f]+ <(\w+)>"


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "list_file", type=argparse.FileType(), help="List file (disassembly)"
    )
    parser.add_argument(
        "callee_pattern", help="Regular expression for the callee function name."
    )

    return parser.parse_args()


def unmangle_names(funcs):
    """
    Unmangles a list of c++ names
    """
    callers = list(f[0] for f in funcs)
    callees = list(f[1] for f in funcs)

    # We dont want to display function parameters' types as it increases the
    # output length too much
    cppfilt = ["arm-none-eabi-c++filt", "--no-params"]

    command = cppfilt + callers
    callers = subprocess.check_output(command).decode().splitlines()

    command = cppfilt + callees
    callees = subprocess.check_output(command).decode().splitlines()

    return list(zip(callers, callees))


def main():
    args = parse_args()

    current_function = "none"

    callees = set()

    for line in args.list_file:
        match = re.search(NAME_PATTERN, line)
        if match:
            current_function = match.group(1)

        if re.search(args.callee_pattern, line):
            func_name = re.search("<(\w+)", line)
            if func_name is None:
                continue
            func_name = func_name.group(1)

            if current_function != func_name:
                callees.add((current_function, func_name))

    callees = unmangle_names(callees)

    # Sort by called functions then callees
    callees.sort(key=lambda s: (s[1], s[0]))

    # Find longest function name
    longest_function_len = max(len(f) for f, _ in callees)

    for current_function, func_name in callees:
        print(
            "{} -> {}".format(current_function.ljust(longest_function_len), func_name)
        )


if __name__ == "__main__":
    main()
