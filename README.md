# Master board
[![Build Status](https://travis-ci.org/cvra/master-firmware.svg?branch=master)](https://travis-ci.org/cvra/master-firmware)

This is the firmware for the "master" board of the robot, which is used for hard realtime tasks such as control and odometry.
It runs on an Olimex E407 board, and communicates with the embedded PC via Ethernet/IP.

# Quickstart
This requires a working ARM toolchain and OpenOCD.
By default it assumes you are using a BusBlaster V2.
You can change this in oocd.cfg.

```
    git submodule init
    git submodule update

    ./packager/packager.py
    make dsdlc
    make
    make flash
```

Now the board should be pingable at 192.168.0.20.

# Kernel panics
If there is a kernel panic, the board will reboot and turn on a green led.
The panic reason is available via the USB shell or via API.

# Deploying on the robots
To deploy on the robots we use Fabric which allows to remotely flash the binary.
You can install it by running `sudo pip install fabric`.
The script will build the firmware, copy it to the robot and flash the board using the embedded JTAG probe.

To deploy to a robot run : `fab debra deploy`.
To deploy to several robot and a local virtual machine: `fab debra caprica vm deploy`.
To list all available commands: `fab -l`

