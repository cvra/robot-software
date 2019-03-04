# Master Board

This is the firmware for the "master" board of the robot, which is used for hard realtime tasks such as control and odometry.
It runs on an Olimex E407 board or a ST Nucleo F429ZI, and communicates with the embedded PC via Ethernet/IP.

### Quickstart
This requires a working ARM toolchain and OpenOCD.
It also requires CVRA's packager system, you can install it by running `sudo pip3 install cvra-packager==1.0.0`.
By default it assumes you are using a ST-Link V2. You can change this in the Makefile.

```bash
    git submodule update --init --recursive
    packager
    make protoc
    make dsdlc
    make
    make flash
```

Now the board should be pingable at 192.168.3.20.

Build and run unit tests

```bash
    packager
    make protoc
    make dsdlc
    mkdir build
    cd build
    cmake ..
    make check
```

### Kernel panics
If there is a kernel panic, the board will turn on all LEDs and continuously print debug information over UART3 at 921600 baud.

## Hardware configuration

The firmware for the two robots is the same.
The hardware jumper `ROBOT_SELECT` on the Nucleo shield tells the software on which robot it runs.
Jumper position 1 selects *Order*, jumper position 2 selects *Chaos*.
The default configuration (i.e. no jumper present) selects *Order*.
The configuration of both robots is compiled, the right one is selected at runtime based on the jumper configuration.
