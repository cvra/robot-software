# Drive Board

This is the firmware for the "drive" board of the rover.
Used to control the rover 6 steered wheels.

### Quickstart
This requires a working ARM toolchain and OpenOCD.
It also requires CVRA's packager system, you can install it by running `sudo pip3 install cvra-packager==1.0.0`.
By default it assumes you are using a ST-Link V2. You can change this in the Makefile.

```bash
    git submodule update --init --recursive
    packager
    make dsdlc
    make
    make flash
```

### Kernel panics
If there is a kernel panic, the board will turn on all LEDs and continuously print debug information over UART3 at 921600 baud.
