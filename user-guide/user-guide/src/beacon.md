# Beacon board

The Beacon board is a 42x30mm with a dedicated STM32F4 microcontroller and the following features:

- CAN interface with communication over UAVCAN.
- CAN bootloader for easy firmware update over the bus.
- An Ultra-Wide Band (UWB) module from Decawave for communication and beacon to beacon distance measurement
- Molex Picoblade connection for wiring CAN in daisy chain.
- Micro USB connector for debugging and flashing via DFU.
- SWD connector for flashing and debugging, with UART exposed on the same connector.
- 4x M2 holes for mounting.
- Costs < 50 USD in components.

Still a work in progress.
Currently basic communication is implemented and distance measurement at low update rate.

![Beacon board, our WIP global positioning system](./images/beacon-board.jpg)

## Links
- [Hardware](https://github.com/cvra/uwb-beacon-board) including KiCad files, the schematics in PDF, and gerber files.
- [Software](https://github.com/cvra/robot-software/tree/master/uwb-beacon-firmware) using ChibiOS RTOS/HAL, and UAVCAN for communication.
- [Bootloader](https://github.com/cvra/can-bootloader) based on libOpenCM3 and a custom lightweight protocol.
