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
