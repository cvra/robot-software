---
freshness:
  - owner: antoinealb
    reviewed: 2021-10-07
---

# Install the ARM compiler 

This document how to install the compiler tools commonly used to program our microcontrollers.
Follow the instructions for your platform, as well as the one in common for many platforms. 

## macOS

1. Make sure the developer tools are installed.
     Homebrew should prompt you to install them if needed.
1. Install the [Homebrew](https://brew.sh) package manager.
1. Run the following commands, which will install the latest version of the toolchain.

```
:::bash
$ brew tap PX4/homebrew-px4
$ brew update
$ brew install gcc-arm-none-eabi
```

You can now check that the compiler is installed. Don't worry if your version does not exactly match the one in this example.

```
:::bash
$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors) 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 224288]
Copyright (C) 2014 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

## Linux

The instructions here depend on the particular Linux distribution, the instructions here were tested on Ubuntu 20.04.

```
:::bash
$ apt update
$ apt install gcc-arm-none-eabi build-essential
```

You can then check that the compiler is installed:

```
$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (15:9-2019-q4-0ubuntu1) 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599]
Copyright (C) 2019 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

## Installing Python tools

You will need a few tools to be installed in order to compile the software.
First make sure that `python3` is installed on your system.
Then, install the required python packages by running the following command:

```
$ pip install \
    git+https://github.com/cvra/packager \
    "git+https://github.com/cvra/can-bootloader#egg=client&subdirectory=client/" \
    "git+https://github.com/cvra/CAN-USB-dongle-fw#egg=tools&subdirectory=tools/" \
    msgpack-python
```
