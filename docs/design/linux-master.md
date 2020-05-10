---
title: Running master-firmware on Linux
author: "antoinealb"
---

# Running master-firmware on Linux

## Background

Currently, most of our high level code runs on an STM32F4 microcontroller, running on a Nucleo board.
It is responsible for control of the wheelbase, communication, HMI, etc.
The board has relatively low RAM for all the functionality that we want: everything has to be packed in 256 kB of RAM.

On the other hand, we have now modern Single Board Computers (SBCs) such as the Raspberry Pi which pack several GB of RAM in a similar footprint.

## Objective

Replace the Nucleo running ChibiOS by a Raspberry Pi running Linux and custom software.

## Risks

In the past, the club tried to use Linux in the robots, which was largely [a failure](https://cvra.ch/blog/2016/goldorak-post-mortem).
However, those projects were trying to do something sensibly more complex than our proposal: typical solutions involved dynamic languages (Python), complicated multi-process solutions (ROS) and complex algorithms (navigation stack).
The new approach is much simpler as more of the code resides in a single binary, which is still written in C++ and compiled on the developer's laptop.
The approach is more similar to a very fast microcontroller than it is to a normal Linux-based robot.

Another issue we had in the past was the reliance on manual processes for building and deploying software on the embedded computer.
A lot of the time it was required to SSH into the robot to start a script, and some of the time editing code on the robot directly was done.
This can be avoided by restraining ourselves to a very minimalist Linux distribution, such as Buildroot.
The image running on the robot would not have development tools, which reduces the human interest in running those issue.

The last remaining risk is the reliability one: a very nice features of microcontrollers is that they do not suffer from boot disk corruption and that they are quick to restart in case of a bug.
The first issue can be mitigated by using modern filesystems (such as ext4) which are resilient to that type of issues.
The second one is also mitigated by having a custom Linux distribution which can be optimized for boot speed.
The [internet](http://himeshp.blogspot.com/2018/08/fast-boot-with-raspberry-pi.html) mentions it is possible to get boot time down to 4 seconds.
One very explicit goal of this project is that **the time from power on to wheelbase control should be 10 seconds max.**
This follows from the fact that the bootloader installed on the motor control board take 10 seconds to boot, and the robot cannot be controlled faster than this.

## Rough technical design

This project can be divided in several parts: hardware, Linux setup and software.

On the hardware side, once a board is picked (probably a Pi 3 or 4 for convenience), an extension board needs to be designed.
This extension board would be plugged in the extension connector and provide the following services:

* CAN adapter, such as Microchip's MCP2515 (< 2$)
* Expose the GPIO required for the front panel.
    Note that if not enough GPIOs are exposed, an I2C GPIO expander is trivial to use under Linux.
* Reading the encoders from the wheel base.
    This could be done using a separate microcontroller on a CAN or SPI bus.
* Interfacing our current LCD to the chosen board.
    This required both a SPI interface (for the display) and an I2C interface (for the touchscreen controller).
* Provide power to the Pi.
    Raspberry Pis can draw [up to 3A](https://www.raspberrypi.org/documentation/faqs/#pi-power) on their 5V lines.
    This is way more than what we used on the Nucleo, and therefore it cannot be powered through the bus.
    Therefore, the board would require its own DC/DC to go from Vbat to 5V.

On the Linux setup side, we would build a custom Linux distribution using the [Buildroot](http://buildroot.org) framework.
This framework enables easy customization of target system, while being very reproducible.
Some customization to the Linux kernel are required: in particular enabling drivers for things like the CAN driver and the LCD screen.
In addition to this, we need to write [Device Tree Overlays](https://www.raspberrypi.org/documentation/configuration/device-tree.md) so that the kernel knows about our custom hardware.

We will need a way to deploy our software to the Linux system and to start it at boot.
The deploy can be done using SSH, while the automatic starting can be done using the init system (we will use systemd).
Some scripts will need to be written to make that process as painless as possible.

Finally, we need to do the following changes to the master firmware code:

1. (in parallel with the rest) Port to CMake build, to use the cross compiler easily.
1. Port all threading code to use Linux threads (with pthread probably).
1. Change code that reads GPIO for front panel to read them using Linux API (sysfs).
1. Change uGFX drivers to use SDL and start on the embedded touchscreen.
1. Rewrite the shell to listen on a TCP port instead.

## Opportunities that this enables

### Rewriting key code to use dynamic memory allocation

* GOAP
* motor manager
* UDP topic broadcasting

### Enhanced logging

The first step would be to write the program's logs to stdout to be caught by systemd.
However, it could be interesting to have an easy way to retrieve logs from the program, for example through an HTTP server.

### Migrating the HMI to Qt

Currently, we use uGFX to display information on the onboard screen, which is a very lightweight system intended to run on microcontrollers.
However, it is also not very friendly to use.
On the other hand, modern frameworks such as Qt have tools such as interface designers that would make the process of designing new panels much easier.

### Static analysis

* Locks
* Use after free

### Simulation

### Wireless debug

## Alternatives considered

TODO: Explain why not those

* Rewriting master-firmware in ROS
* F7
* keeping the same platform
