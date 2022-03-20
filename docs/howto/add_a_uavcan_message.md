---
freshness:
  - owner: antoinealb
    reviewed: 2021-10-25
---

# Handle new UAVCAN data types

Things you will need for this tutorial:

* A [working arm-none-eabi compiler setup](../howto/install_arm_gcc.md)
* A can-io board
* A CAN cable
* A CVRA USB dongle
* two 120 Ohm resistors

The goal of this tutorial is the classical "Hello World" in embedded sytems: blinking a LED.
However, we want this to happen through CAN, in order to learn how to implement new services.

## Disabling the automatic LED blinking

Go to the `can-io-firmware`, and edit `main.c` to disable the blinker thread:

```
:::c++
int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    // comment the following line
    // blinker_start();
```

Then, rebuild the new firmware for the CAN IO board, by running the following commands:

```
:::bash
cd can-io-firmware/
packager
make dsdlc
make USE_BOOTLOADER=yes
```

We can now power the bus through USB, and to flash the board:

```
:::bash
# Note that the path to the serial port (/dev/...) changes from computer
# to computer.
can_dongle_power /dev/tty.usbmodem3031 on

# Check that the board is detected
bootloader_read_config -p /dev/tty.usbmodem3031 --all
```

You should get the following output.
If you get an error you can retry, as it can get stuck.

```
:::json
{
    "1": {
        "ID": 1,
        "application_crc": 1298708532,
        "application_size": 44808,
        "device_class": "can-io-board",
        "name": "foobar2000",
        "update_count": 2
    }
}
```

If you get a message "Permission Denied", it means your user does not have permissions to access serial ports.
You can confirm this by running the same command, but with `sudo` to become admin first (`sudo can_dongle_power /dev/tty.usbmodem3031 on`).
You will be asked for your admin password; nothing will be shown on the screen, its normal for `sudo`.
If it works, you can then give your user access to the serial ports by running `sudo usermod -a -G dialout $USER`, then rebooting (tested on Ubuntu).

This is the configuration of the bootloader for this board.
It is not erased when reflashing and can be read from the application.

* **ID** is the address of the board, which must be unique on the bus and must be between and 127.
* **application_crc** and **application_size** are used by the bootloader to detect if the firmware is corrupted, for example missing parts of the firmware.
* **device_class** is the name of the hardware model, and is used to check if we are flashing new code on the correct board, to avoid damage.
* **name** is a human readable name for this board, to help diagnose which board we are talking to (`left-wheel` is way easier to guess 42). For now it is set to the default, `foobar2000`.
* Finally, **update_count** is the number of times a new firmware was flashed by the bootloader, we use it to find if there are issues with old boards.

Now that we know the ID of our board we can flash it with our fresh firmware:

```
:::bash
bootloader_flash --port /dev/tty.usbmodem3031 \
    --binary build/ch.bin \
    --base-address 0x08003800 \
    --device-class can-io-board \
    --run \
    1
```

As you can see the bootloader requires a few flags to flash the board.
Fortunately most of them do not change very often, or are easy to find.

* `--port` is the name of the serial port on which the CAN dongle is connected.
* `--binary` is the path to the binary we want to flash.
* `--base-address` is the address of the beginning of the application memory.
    You can find it in the linker script [`can-io-firmware/linker/STM32F302x8_bootloader.ld`](https://github.com/cvra/robot-software/blob/master/can-io-firmware/linker/STM32F302x8_bootloader.ld#L8), look for `flash0`.
* `--device-class` must match the one stored in the config.
* `--run` asks the board to run the application once the firmware upload is done.

Finally, we put the list of board IDs to update (just `1` here).

You should get a progress bar, with `Verifying firmware... OK` at the end.
Your application will start and the LED should be off.

## Defining a new UAVCAN message type

We will create new message type that will be used to ask the board to turn the LED on or off.
Create a new file in `uavcan_data_types/cvra/io/200.LEDCommand.uavcan` with the following content.
This defines a **service**, which is made of a request and a response.
The request here contains just a single boolean, to indicate the desired LED state and the response a single integer for demo purpose.

```
bool led_status
---
int32 data
```

Now we must run `make dsdlc` again.
This will generate the C++ code required to serialize and interpret our new message type.

## Handling the new message type in our application

Create a new C++ file `can-io-firmware/src/uavcan/LEDCommand_handler.cpp` with the following content

```
:::c++
/* For accessing the GPIO functions */
#include <hal.h>

/* UAVCAN interface */
#include <uavcan/uavcan.hpp>

/* Code generated from our .uavcan file */
#include <cvra/io/LEDCommand.hpp>

/* This function is called every time we receive a request for our newly
 * defined LedCommand service.
 */
void LEDCommand_handler(
    const uavcan::ReceivedDataStructure<cvra::io::LEDCommand::Request>& req,
    cvra::io::LEDCommand::Response& rsp)
{
    /*  req contains the incoming data, while rsp will be sent back as a reply
     *  to this request. */

    /* Set the LED status to the requested value */
    if (req.led_status) {
        palSetPad(GPIOA, GPIOA_LED);
    } else {
        palClearPad(GPIOA, GPIOA_LED);
    }

    /* Reply with an example data. */
    rsp.data = 42;
}
```

Also create the associated header, `can-io-firmware/src/uavcan/LEDCommand_handler.hpp`.
In this file we will put the prototype of our handler:

```
:::c++
#pragma once

#include <uavcan/uavcan.hpp>
#include <cvra/io/LEDCommand.hpp>

void LEDCommand_handler(
    const uavcan::ReceivedDataStructure<cvra::io::LEDCommand::Request>& req,
    cvra::io::LEDCommand::Response &rsp);
```

We can then add this new file to the build system, in `can-io-firmware/package.yml`:

```
:::yaml
# [...]
target.arm:
# [...]
  - src/uavcan/ServoPWM_handler.cpp
  - src/uavcan/DigitalInput_pub.cpp
  - src/uavcan/LEDCommand_handler.cpp # add this line
# [...]
```

Now you can regenerate the Makefiles (`packager`) and rebuild the binary:

```
:::bash
packager
make USE_BOOTLOADER=yes
```

Finally, reflash it.
You can use the same command as before, but you need to power cycle the board first, to put it back in bootloader mode.

## Driving our LED from Python

Now it is time to drive our LED from our computer, using a simple Python script.
Put the following in `can-io-firmware/blink.py`.
You can run it by running `cd can-io-firmware && python blink.py`.

```
:::python
import uavcan

# Reads our new custom message definition
uavcan.load_dsdl("../uavcan_data_types/cvra")

# Creates a UAVCAN device with address 123.
# As before, you might have to change the serial port to which your CAN adapter
# is connected.
node = uavcan.make_node("/dev/tty.usbmodem3031", node_id=123)
BOARD_ID = 1

led_status = True

# This function is called every time we receive a response from the board, it
# simply prints the data we get back
def led_command_reply_received(event):
    print("reponse data", event.response.data)


while True:
    try:
        # Process UAVCAN messages for 1 second, then returns.
        node.spin(1)

        # Sends a request to set the LED status to our target board.
        node.request(
            uavcan.thirdparty.cvra.io.LEDCommand.Request(led_status=led_status),
            BOARD_ID,
            led_command_reply_received,
        )

        # Finally, invert the LED state so that it changes once a second.
        led_status = not led_status
    except uavcan.UAVCANException as ex:
        print("Node error:", ex)
```

## Conclusion

In this tutorial, we saw how to define and use a custom message type for a new application.
We saw how this integrate in CVRA's tools (bootloader and build system).

The code for this tutorial is on [Github](https://github.com/cvra/robot-software/tree/uavcan-tutorial).
You can find more UAVCAN tutorials on their [homepage](https://legacy.uavcan.org/).
