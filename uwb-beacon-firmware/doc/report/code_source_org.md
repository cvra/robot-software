The code of this project can be separated into three different categories: libraries, portable code and platform specific code.
Libraries are Open Source code that are re-used from other projects.
Portable code is generic code that does not contain anything specific to this processor or board, but is only useful in this project.
It can be moved into a library if another project requires it later.
Portable code is usually well covered by unit tests (see the `tests` folder), where non portable code must usually be tested "by hand".

# Libraries

Most of those libraries have READMEs that describe them in much more detail.

* `chibios-syscalls` provides a few extensions for ChibiOS (our RTOS) for malloc and printf mostly.
* `crc` implements some common checksums algorithms.
* `decadriver` is provided by Decawave to interact with their product.
* `eigen` is a C++ matrix manipulation library used for the EKF.
* `parameter` provides a unified interface for the application to declare parameters.
* `cmp`,  `cmp_mem_access` and `parameter_flash_storage` are used to store the parameters in non volatile memory.
* `msgbus` implements a software message bus (publisher/subscriber pattern), similar in concept to D-bus or ROS.
* `libuavcan` is the official UAVCAN stack.
* `test-runner` is used only to provide an environment during unit testing.

# Portable code

* `MadgwickAHRS.{c,h}` contains the implementation of the Madgwick filter.
* `lru_cache.{c,h}` contains a generic implementation of a Least Recently Used (LRU) cache.
* `mpu9250.{c,h}` contains a portable implementation of a driver for the Invensense MPU9250 IMU.
* `state_estimation.{cpp,h}` contains the code for the state estimator (uses ekf.hpp).
* `ekf.hpp` contains a generic implementation of an EKF using C++ templates.
* `uwb_protocol.{c,h}`  contains a portable implementation of the UWB protocol.

# Non portable code

* `ahrs_thread.{c,h}` contains the thread that runs the Madgwick filter.
* `anchor_position_cache.{c,h}` contains the thread that stores the position of the anchors and provides them to the other threads.
* `imu_thread.{c,h}` contains the thread that runs the IMU driver.
* `ranging_thread.{c,h}` contains the thread responsible for doing range measurements and other radio operations.
* `state_estimation_thread.{cpp,h}` contains the thread that runs the state estimation.
* `usbconf.{c,h}` contains the USB setup code.
* `main.{c,h}` contains the entrypoint of the application.
* `cmd.{c,h}` contains the command line interface shown on USB.
* `decawave_interface.{c,h}` contains the low level bindings between Decawave's library and our code.
* `board.{c,h}` contains GPIO contains GPIO configuration.
* `bootloader_config.{c,h}` contains code for interacting with the CAN bootloader used on some boards.
* `exti.{c,h}` contains the external interrupt configuration and handlers.
* The `uavcan` folder contains code related to moving messages from the application to the CAN bus and vice versa.
