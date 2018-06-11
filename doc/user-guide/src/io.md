# IO board

The IO board is a 23x15mm board with a dedicated STM32F3 microcontroller and the following features:

- CAN interface with communication over UAVCAN for IO control (read / write).
- CAN bootloader for easy firmware update over the bus.
- 11 GPIOs exposing:
    * Digital outputs, including timer channels for PWM,
    * Digital inputs, including timer channels for pulse counting,
    * Analog inputs (ADC).
- Communication busses such as I²C and SPI.
- Molex Picoblade connection for wiring CAN in daisy chain.
- SWD connector for flashing and debugging, with UART exposed on the same connector.
- Costs < 10 USD in components.

Current application software allows a general purpose usage exposing basic IO control over UAVCAN: PWM output and digital inputs.

![An IO board with USB type A connector behind for scale](http://www.cvra.ch/images/technologies/io-board.jpg)

## Links
- [Hardware](https://github.com/cvra/can-io-board) including KiCad files, the schematics in PDF, and gerber files.
- [Software](https://github.com/cvra/robot-software/tree/master/can-io-firmware) using ChibiOS RTOS/HAL, and UAVCAN for communication.
- [Bootloader](https://github.com/cvra/can-bootloader) based on libOpenCM3 and a custom lightweight protocol.
