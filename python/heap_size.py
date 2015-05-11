#!/usr/bin/env python3
import subprocess
import sys

RAMSIZE = 111000

command = "arm-none-eabi-size {}".format(sys.argv[1]).split()
result = subprocess.check_output(command).splitlines()[1]
result = result.split()
data, bss = int(result[1]), int(result[2])

print("Heap space left: {} bytes".format(RAMSIZE - (data + bss)))
