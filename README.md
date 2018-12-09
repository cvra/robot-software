# Robot software stack

[![Build Status](https://travis-ci.org/cvra/robot-software.svg?branch=master)](https://travis-ci.org/cvra/robot-software)

This repo contains all the software used on our robots:
- `can-io-firmware` contains the firmware that runs on the [IO board](http://www.cvra.ch/technologies/io_board.html)
- `motor-control-firmware` contains the firmware that runs on the [motor board](http://www.cvra.ch/technologies/motor_board.html)
- `proximity-beacon-firmware` contains the firmware that runs on the proximity beacon, it's the same code as the motor board but with a different application that is tailored to the needs of our proximity beacon module
- `master-firmware` contains the software that runs on the master board, it interfaces all the other boards over CAN and runs the robot's "intelligence".
- `eurobt` contains documentation and cofiguration files specific to the Eurobot competition
- `minesweepers` contains all code specific to our rover for the Minesweepers competition
    * `minesweepers/doc` contains design documentation
    * `minesweepers/techreport` contains the technical report source files
- `sensor-firmware` contains code running on the [sensor board](https://www.cvra.ch/robot-software/sensor.html)
- `uwb-beacon-firmware` contains code and documentation that runs on the [UWB beacon board](https://www.cvra.ch/robot-software/beacon.html)

Other important software components can be found in this repo:
- `lib` contains all the libraries and building blocks we use on multiple boards, which includes:
    * `lib/can-bootloader` the bootloader that allows us to update our boards (IO and motor) over CAN
    * `lib/ChibiOS` the RTOS/HAL we use on all our boards
    * `lib/uavcan` the CAN communication library we use on all our boards
    * `lib/error` a logging library
    * `lib/parameter` a library to create and manage configurations of boards
    * `lib/msgbus` a publish/subscribe library for inter thread communication
    * and more.
- `tools` groups all tools we use to develop on the robot including:
    * `tools/pid-tuner` a GUI to tune PID gains of motor boards over CAN, written using Python and Qt
    * `tools/studio` a set of introspection tools written in Python and Qt to debug our robots
- `uavcan_data_types` contains the custom message definitions (DSDL) for the UAVCAN communication protocol
- `ci` groups scripts and Docker files for our continuous integration server
- `user-guide` contains high-level documentation about software and electronics components used on our robots

The [user guide](http://cvra.ch/robot-software) is generated using mdbook from [doc/user-guide](/doc/user-guide)

# Coding style

We use `clang-format` (tested with version 7 or greater) to enforce proper source code formatting.
You can use the `format-all.sh` script in the root directory to format the whole source tree.
You can also use `clang-format -i --style=file src/foo.c` to format a particular file.
Finally, some editors include support for `clang-format` through plugin, check for yourself.
