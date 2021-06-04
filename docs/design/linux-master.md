---
title: Porting master-firmware to Linux
author: "antoinealb"
freshness:
  - owner: antoinealb
    reviewed: 2020-11-18
---

# Running master-firmware on Linux

## Background

Currently, most of our high level code runs on an STM32F4 microcontroller, running on a Nucleo board.
It is responsible for control of the wheelbase, communication, HMI, etc.
The board has relatively low RAM for all the functionality that we want: everything has to be packed in 256 kB of RAM.

On the other hand, we have now modern Single Board Computers (SBCs) such as the Raspberry Pi which pack several GB of RAM in a similar footprint.

## Objective

Replace the Nucleo running ChibiOS by a Raspberry Pi running Linux and custom software.
This simplifies our setup, and makes it more accessible to new members by using common hardware and operating system.

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

## Technical design

This project can be divided in several parts: hardware, Linux setup and software.

### Hardware

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

### Linux setup

On the Linux setup side, we would build a custom Linux distribution using the [Buildroot](http://buildroot.org) framework.
This framework enables easy customization of target system, while being very reproducible.
Some customization to the Linux kernel are required: in particular enabling drivers for things like the CAN driver and the LCD screen.
In addition to this, we need to write [Device Tree Overlays](https://www.raspberrypi.org/documentation/configuration/device-tree.md) so that the kernel knows about our custom hardware.

We will need a way to deploy our software to the Linux system and to start it at boot.
The deploy can be done over the network, while the automatic starting can be done using the init system (we will use systemd).
To make this as streamlined as possible, the build system will generate a package for [opkg](https://openwrt.org/docs/guide-user/additional-software/opkg).
A binary running on the robot can receive this package over the network and install it.
This is a very flexible mechanism as everything is contained in the package and can be easily re-used, for example for OS upgrades.

### Software changes

Finally, we need to do the following changes to the master firmware code:

1. (in parallel with the rest) Port to CMake build, to use the cross compiler easily.
1. Port all threading code to use Linux threads (with pthread probably).
1. Change code that reads GPIO for front panel to read them using Linux API (sysfs).
1. Read encoders from CAN
1. Change uGFX drivers to use SDL and start on the embedded touchscreen.
1. Rewrite the shell to listen on a TCP port instead.

## Alternatives considered

The first alternative considered would be to keep the same platform.
This worked fine for the first years, but is a cost that much be paid each time we want to add a complex feature.
We also learnt enough about embedded Linux in the meantime to have a good chance of having something stable now.

Another option would be to migrate to a bigger microcontroller such as the STM32F7 series.
Those offer a lot of RAM and could still be used with ChibiOS, reducing the porting effort.
While this makes development of complex applications easier, it is a small bandaid, where we would be better served by making a bigger leap to another platform.
Migrating to Linux is also a one time cost, while upgrading to the next microcontroller family bears a significant cost.

The last option would be to rewrite all the high level code of the robot, perhaps using a framework such as [ROS2](https://index.ros.org/doc/ros2/).
However, this is a much bigger endeavour, with a lot of risks.
We know that our current wheelbase stack is well performing and a lot of effort was invested into it.
Rewriting it from scratch is probably not a wise use of our (limited) time.

## Opportunities enabled by this proposal

### Rewriting libraries to use dynamic memory allocation

Some of the code running on master firmware is made more complicated or limited by the fact that we currently cannot use dynamic memory.
Systems that would benefit from this include:

* GOAP, whose implementation is made more complicated by the refusal to use dynamic memory allocation.
    Note that we could also afford to explore bigger plans if we had more RAM, making it easier to write strategies.
* Bus enumerator, which could be replaced by a very simple `std::unordered_map`.
    We could also remove the need to register each board in advance.
* UDP topic broadcasting, which has a queue of bounded size and could just use a unbounded FIFO.

Such examples are many in our code base, and should be easy to convert.

### Enhanced logging

The first step would be to write the program's logs to stdout to be caught by systemd.
However, it could be interesting to have an easy way to retrieve logs from the program, for example through an HTTP server.

### Better HMI

Currently, we use uGFX to display information on the onboard screen, which is a very lightweight system intended to run on microcontrollers.
However, it is also not very friendly to use.
On the other hand, modern frameworks such as Qt have tools such as interface designers that would make the process of designing new panels much easier.

We could also migrate from our low resolution display to a better one using the MIPI interface.
This interface is used for high bandwidth display information but put restrictions on the maximum cable length.

### Static analysis

Using modern compilers along with a more standard setup would allow us to do more complex static analysis.
For example Clang can check at compile time that [locks are correctly used to protect shared variables](https://clang.llvm.org/docs/ThreadSafetyAnalysis.html).
This could increase the safety of our code, at a very low cost.

### Wireless debug

Some Raspberry Pi boards have onboard support for Wifi.
We could potentially have the robot generate a hotspot, to which we could connect for debugging while the robot is running.

## Tasks & effort estimates

Task                                                                      | Done | Owner      | Work estimate
--------------------------------------------------------------------------|-|-----------|---------------
**Electronics**                                                                      |   |            | **Total: 6**
Schematic for hardware extension for Pi                                              | ✔️ |            | 2
Routing the PCB                                                                      | ✔️ |            | 2
PCB Assembly                                                                         | ✔️ |            | 1
Write a firmware to read wheelbase encoders and send it over CAN (running on nucleo) | ✔️ | nuft       | 1
**Linux setup**                                                                      |   |            | **Total: 10**
Get Buildroot to boot on chosen SBC                                                  | ✔️ | antoinealb | 1
Modify kernel to for our custom hardware                                             | ✔️ |  antoinealb | 2
Setup ethernet networking & SSH                                                      | ✔️ |            | 1
Optimize boot time to stay under 10 seconds                                          | ✔️ |             | ??
Deployement scripts and autostart of CVRA software                                   | ✔️ | antoinealb | 3
**Master firmware modifications**                                                    |   |             | **Total: 11**
Port enough of the master-firmware to have the uavcan part running on Linux          | ✔️ | antoinealb | 3
Rewrite debug shell to listen over TCP instead of UART                               |   |             | 2
Read encoders from CAN instead of from hardware directly.                            | ✔️ |            | 1
Port rest of the code required for wheelbase control.                                | ✔️ |            | 1
Manipulate LEDs using sysfs instead of ChibiOS API                                   | ✔️ | antoinealb | 1
Reads front panel buttons using libgpiod instead of ChibiOS API                      | ✔️ |            | 1
Port rest of the code required to play a match                                       |   |            | 2
Change HMI backend to use SDL API (might require Linux setup on Pi)                  | ✔️ | antoinealb | 1
