#!/usr/bin/env python3
import subprocess
import sys

RAMSIZE = 111000

command = "arm-none-eabi-nm --numeric-sort --print-size {}".format(sys.argv[1]).split()
result = subprocess.check_output(command).decode().splitlines()

heap_base = [s for s in result if "__heap_base__" in s][-1]
heap_end = [s for s in result if "__heap_end__" in s][-1]

heap_base = int(heap_base.split()[0], 16)
heap_end = int(heap_end.split()[0], 16)


print("Heap space left: {} bytes".format(heap_end - heap_base))
