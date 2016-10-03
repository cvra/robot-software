# Robot software stack

## Master board
[![Build Status](https://travis-ci.org/cvra/master-firmware.svg?branch=master)](https://travis-ci.org/cvra/master-firmware)

This is the firmware for the "master" board of the robot, which is used for hard realtime tasks such as control and odometry.
It runs on an Olimex E407 board or a ST Nucleo F429ZI, and communicates with the embedded PC via Ethernet/IP.

### Quickstart
This requires a working ARM toolchain and OpenOCD.
It also requires CVRA's packager system, you can install it by running `sudo pip3 install cvra-packager==1.0.0`.
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
If there is a kernel panic, the board will reboot and turn on a green led.
The panic reason is available via the USB shell or via API.
