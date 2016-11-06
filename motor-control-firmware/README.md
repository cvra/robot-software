# Motor board firmware

## Building

```
$ ./build.sh
```

## Flashing

Flashing with CAN bootloader requires the `cvra-bootloader` python package.
```
$ bootloader_flash -p TTY -b BINARY -r -a 0x08003800 -c motor-board-v1 IDs
```

# GDB Debugging

Set `USE_BOOTLOADER = no` in the Makefile, then build & lauch openocd:
```
$ ./build.sh
$ openocd -f openocd.cfg
```

Use GDB from another session:
```
$ arm-none-eabi-gdb
(gdb) load # flash app
(gdb) break main
(gdb) continue
...
```

Note: The GDB arguments are `.gdbinit` file in the motor-control-firmware directory. If you want to launch GDB from another location, then you need to pass following arguments:
```
$ arm-none-eabi-gdb --eval-command='target remote localhost:3333' build/motor-control-firmware.elf
```

## Run tests

```
$ ./test.sh
```
