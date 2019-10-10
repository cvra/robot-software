# Hardware in the loop

# Requirements

* SocketCAN for the parts that emulate a board.
    That means those binaries can only be used on Linux.
* gRPC for inter process communication.
    This was testing with version 1.24.1.
    See [the quick start guide](https://grpc.io/docs/quickstart/cpp/) for explanations.
* Protoc version 3 or more.

## Building

From the root of the project (robot-software), **NOT** from this directory, run the following:

```
mkdir build
cd build
cmake ..
make
```

## Adding a virtual SocketCAN interface

```bash
lib/uavcan/libuavcan_drivers/linux/scripts/uavcan_add_vcan vcan0

# check its existence
ip link show dev vcan0

# to delete it
ip link delete vcan0
```

