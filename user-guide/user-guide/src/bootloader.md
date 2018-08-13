# CAN bootloader

The bootloader is a general purpose bootloader, that we use on all our CAN boards.

It allows us to flash them with application firmwares over CAN, thus removing the need to intervene directly on the boards (via debugger).
And it stores a small config in the microcontroller's flash that makes it easily trackable (ID, board type).

Its main features are:

- Firmware flashing over CAN, without intervention on the board directly.
- Firmware flashing is fast, a typical firmware (few hundred kilobytes) is written in a few seconds over a 1Mbps CAN interface.
- Parallel firmware flashing (using multicast) when several boards are flashed with the same firmware.
- Board tracking through small config containing ID, name, board type, and number of times the flash was erased.
- On power up, it waits for a few seconds for the user to input commands before jumping to the application.
- Application code is checked by CRC at boot, invalid applications are not loaded.

Currently supported platforms are:

- All our boards at CVRA: motor, IO, sensor, and beacon boards.
- ST Nucleo STM32F103RB board
- ST Nucleo STM32F334R8 board
- Olimex E407 board (with STM32F407 onboard)

## Links
- [Bootloader source code](https://github.com/cvra/can-bootloader)
- [Supported platforms](https://github.com/cvra/can-bootloader/tree/master/platform)
