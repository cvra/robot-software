# Motor board firmware

## Building

```
$ packager && make dsdlc && make
```

## Flashing

Flashing with CAN bootloader requires the `cvra-bootloader` python package.
```
$ bootloader_flash -p TTY -b build/ch.bin -r -a 0x08003800 -c motor-board-v1 ID
```
Note:
- The tty of the CAN dongle is usually `/dev/tty.usbmodem301` under macOS.
- Under Linux you can also specify a CAN interface using the option `-i can0`.
- You can specify multiple IDs to update multiple boards at once.

# GDB Debugging (without bootloader)

Rebuild without bootloader, flash and lauch openocd:
```
$ make clean && make USE_BOOTLOADER=no
$ make flash
$ openocd -f openocd.cfg
```

Use GDB from another session:
```
$ arm-none-eabi-gdb
(gdb) break main
(gdb) continue
...
```

Note: The GDB arguments are `.gdbinit` file in the motor-control-firmware directory. If you want to launch GDB from another location, then you need to pass following arguments:
```
$ arm-none-eabi-gdb --eval-command='target remote localhost:3333' build/ch.elf
```

## Run tests

```
$ packager
$ mkdir -p build
$ cd build
$ cmake ..
$ make check
```
