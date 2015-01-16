# Master board
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
    make
    make flash
```

Now the board should be pingable at 192.168.0.20.

# Kernel panics
If there is a kernel panic, the board will reboot and turn on a green led.
The panic reason is available via the USB shell or via API.

