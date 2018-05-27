# Sensor board

The Sensor board is a 23x17mm with a dedicated STM32F3 microcontroller and the following features:

- CAN interface over which sensor data are streamed over UAVCAN.
- CAN bootloader for easy firmware update over the bus.
- Distance time-of-flight sensor (VL6180X) can measure distances from 10mm to 100mm with 1mm resolution.
- RGB Color sensor (TCS34725) with external illumination using a white LED
- Molex Picoblade connection for wiring CAN in daisy chain.
- SWD connector for flashing and debugging, with UART exposed on the same connector.
- 2x M2 holes for mounting
- Costs < 25 USD in components.

Current application software only supports the distance sensor.
Color sensor support is work in progress.

## Links
- [Hardware](https://github.com/cvra/tof-sensor-board) including KiCad files, the schematics in PDF, and gerber files.
- [Software](https://github.com/cvra/robot-software/tree/master/sensor-firmware) using ChibiOS RTOS/HAL, and UAVCAN for communication.
- [Bootloader](https://github.com/cvra/can-bootloader) based on libOpenCM3 and a custom lightweight protocol.
