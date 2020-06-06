---
freshness:
  - owner: antoinealb
    reviewed: : 2020-06-05
  - owner: nuft
    reviewed: : 2020-06-05
---
# Master Board

This is the firmware for the "master" board of the robot, which is used for hard realtime tasks such as control and odometry.
It currently is being ported to Linux, and we intend to run it on a Raspberry Pi 4 running Buildroot.
The firmware is **always** cross-compiled: the robot does not have a compiler installed on it.

## Quickstart

The easiest way to get started is with the Dockerfile that contains the cross-compiler.
It will soon be available online, for now we assume it was built with the name `cvra-sdk`.

The following setup has to be done **once**:
```bash
git submodule update --init --recursive
docker run -it -v $(pwd):/src -w /src/lib/nanopb/nanopb/generator/proto cvra-sdk make
mkdir build-docker
docker run -it -v $(pwd):/src -w /src/build-docker cvra-sdk cmake .. -DCMAKE_TOOLCHAIN_FILE=/aarch64-buildroot-linux-gnu_sdk-buildroot/share/buildroot/toolchainfile.cmake
```

Now, to build the package that can be installed on the robot, you need to run the following command.
Note that only this command is required after changing source file.

```bash
docker run -it -v $(pwd):/src -w /src/build-docker cvra-sdk make master-firmware.ipk
```

The resulting file is `build/master-firmware/master-firmware.ipk`.

## Flashing the result to the Pi

**Note:** This is for a Raspberry Pi running our [Buildroot setup](https://github.com/cvra/buildroot), it will not work on Raspian or any other operating systems.

If you connect your computer to the Pi through the USB-C connector, a virtual Ethernet device will be created.
A DHCP server is running on that interface, meaning you should not have to configure anything on the network side.
The Pi will always have `10.1.1.1` as its IP address.
You can then transfer the package to the Pi and install it by running the following commands:

```bash
scp master-firmware/master-firmware.ipk root@10.1.1.1:
ssh root@10.1.1.1 "okg install --force-reinstall master-firmware.ipk"
```

At this point, the code will start, and blink the red led on the Pi.
The package contains all the required configuration to start automatically at boot.
No additional work is required. 
