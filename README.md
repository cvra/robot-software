# Robot software stack

## Master board
[![Build Status](https://travis-ci.org/cvra/robot-software.svg?branch=master)](https://travis-ci.org/cvra/robot-software)

This is the firmware for the "master" board of the robot, which is used for hard realtime tasks such as control and odometry.
It runs on an Olimex E407 board or a ST Nucleo F429ZI, and communicates with the embedded PC via Ethernet/IP.

## Requirements

* GCC-ARM toolchain
* OpenOCD 0.9
* The protocol buffer compiler
* CVRA's packager system (can be installed using `sudo pip3 install cvra-packager`)


### Quickstart
By default it assumes you are using a ST-Link V2. You can change this in the Makefile.

```bash
    git submodule update --init --recursive

    pushd master-firmware
    packager
    make dsdlc
    make config_msgpack
    make
    make flash
```

Now the board should be pingable at 192.168.3.20.

### Kernel panics
If there is a kernel panic, the board will turn on all LEDs and continuously print debug information over UART3 at 921600 baud.
