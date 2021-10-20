---
freshness:
  - owner: antoinealb
    reviewed: 2021-10-20
---


# How we reached our current architecture

## Background & History

In order to understand the current design, I think one must understand where we
are coming from. The club experimented with different platforms over the years,
and every time outgrew those. Every time we switch to a new platform, we must
throw out everything and start again from scratch. To avoid this waste, the
"obvious" solution is to design a system that we will never outgrow, but such a
design is unlikely to succeed as technology and requirements always evolve.

When I joined the club (2008) it was using a board with a single
microcontroller, a fixed number of I/Os and channels for 3 motors with position
feedback and PID control. The architecture was quite rigid, but the motherboard
allowed a bit of customization of the input/output, and when we needed
functionality that could not be implemented on this platform, we built dedicated
boards with small microcontrollers on them, communicating via ad-hoc protocols,
generally over UART or GPIOs. This approach served us well for several years,
but started to show its age (the PID controllers were 20 year old when we moved
away!).

In 2010, we decided to switch to another system. In particular, we wanted to do
polar control of the wheelbase (rather than per-wheel), which the existing
system could not do as it relied on dedicated hardware for PID control. We also
wanted to experiment with onboarding a computer in the robot for computer
vision. The result was made of two parts: a custom board and a Linux computer.
The board had an ATMega as its core. It could control three motors like the old
board, but the PID was done in software, meaning we could do polar control. The
Linux computer was communicating with the board over USB, sending it orders such
as "go to this point". The system worked well enough, but was not very modular:
adding functionality for additional actuators required a lot of code changes. We
were also not too convinced by the platform the computer was using (URBI, a
now-dead programming language for robots).

In 2011, we built our first Debra, the name of our robots with SCARA arms. This
was a massive increase in the number of motors we had to control: we went from 2
PIDs to 12, and they needed coordination. It was clear that our current approach
did not scale to those requirements. The ATmega had to go, and was only used for
one year. Realizing this was wasteful, we committed to a more modular solution,
which we could adapt to each year's requirements. We turned to FPGAs, as they
provide the ultimate modularity: you can change what the hardware is doing
simply by reflashing the FPGA! We still had a computer onboard for tasks like
computer vision, but it never really got used.

The FPGA setup served us well, but it was a nightmare to develop for. FPGAs are
programmed very differently from conventional platforms. To make things worse,
we have to use tools provided by the FPGA vendors, which are pretty bad, non
standard and had some bugs. We stuck with it for a few years, fixing bugs and in
2014, this platform had us win the Swiss championships! However, we needed a
change for several reasons:

-   The FPGAs were too complicated to program[^1],
-   The platform was pretty expensive, meaning we only had three setups (two in
    robots, and one redundant). Developing outside of the robots was impossible.
-   The bugs of the platform made it challenging for reliability,
-   The boards were quite big, which made it mechanically challenging to
    integrate.

We started brainstorming for a new solution in 2015, and this document presents
the current architecture, which is what we have running for now.

## Objective

Provide a platform that can be extended indefinitely to match the requirements
of the robot. Give developers flexibility in choosing the best platform for the
subsystem they are working on.

## Requirements

-   Can drive more than 12 DC motors, since this is what we are replacing
-   Compact
-   Real time requirements
-   Can be used for "10 years" because rebuilding PCBs cost time and money.

## Overview

Unlike previous systems, which were relatively centralized, the new architecture
is made of many systems collaborating to control the robots. While in the past
our robot typically had 2-3 microcontrollers, the new design has \~15!

Linking each microcontroller to each other via a dedicated UART link like we
used to do in the past would be infeasible just by the number of wires and UART
interfaces required. Instead, this design uses a
[field bus](https://en.wikipedia.org/wiki/Fieldbus) shared by all the nodes: a
single physical interface is enough for each node to talk to every other node.

Each microcontroller exposes a very high interface to the rest of the robot.
This is very important to make the system easy to test and reason about: compare
telling a board "turn on pump \#3" to "please set register \#4 to 0xfe".

## Detailed design

The robot's network is based on the CAN (Controller Area Network) protocol. CAN
was originally designed for the automotive industry, where it is used to
communicate between different parts of an engine and/or a car's interior. It was
designed for robustness (safety critical systems depend on it), electrical
resilience (a car emits a lot of electrical noise) as well as wiring simplicity
(wires weigh a lot). CAN transmit data over a
[differential pair](https://en.wikipedia.org/wiki/Differential_signalling), with
the two signals named CANH and CANL.

By itself, CAN is a very simple protocol: it can transmit messages up to 8
bytes, which are tagged with a 29-bit identifier. Any node on the bus can send
and all the nodes will receive all the messages. Therefore, it is common to add
higher level protocols on top of CAN, which provide longer messages, addressing
and message serialization[^2]. We chose UAVCAN for this, which is an emerging
standard aimed at small drones and robots, an application close to ours. It
proposes a nice set of features, and has a good quality reference
implementation. Note that UAVCAN has two versions: the one we use, v0, is
deprecated, and v1, currently in development.

CAN, like most low-level protocols, does not guarantee delivery of messages:
messages can be lost, for example if two different nodes send a message at the
same time. This lead to the introduction of two different message types in
UAVCAN: broadcast and service calls. The first one is simply a node sending a
message to everyone on the network, without expecting a response or a way to
find if a message was dropped. It is well suited for things like sensors or a
motor's current position, where it does not matter if we drop a single message.
The second one is used when we want to have a response, or we want to know if
the message was dropped. We use it mostly for setting parameters on board (PID
gains, board modes and so on). This mode does not guarantee message delivery,
but triggers an error if a response was not received in a given time.

To simplify development, UAVCAN can automatically generate code to switch
between human-readable formats and representation on the CAN bus. Messages are
described in a special language
([example](https://github.com/cvra/robot-software/blob/master/uavcan_data_types/cvra/motor/feedback/20030.MotorPosition.uavcan)),
and C++ or Python code is generated from that.

When working with a large number of devices, software update becomes a
challenge. We used to do that by connecting a JTAG probe to the target
microcontroller, but this would become intractable with so many
microcontrollers, some of which were not reachable without disassembly. We
decided to develop an in-band method of programming, which uses the same CAN
network that we used in normal operation to download updates. When powering up
the robot, boards wait for software update messages for 10 seconds before
proceeding to normal operations. The detailed design can be found in
[cvra/can-bootloader](https://github.com/cvra/can-bootloader).

### Available modules

The first type of board we designed, and still the most commonly used one is the
[motor control board](https://github.com/cvra/motor-control-board) (2015). It
allows the control of a single DC motor, with control loops for controlling in
torque, speed or position. It has inputs for two different quadrature encoders
for position sensing. Originally we wanted to be able to use it as an
alternative means of controlling RC servos, but this turned out to be
unnecessary. It was also re-used with a different firmware for our opponent
detection beacon. The
[API](https://github.com/cvra/robot-software/blob/master/uavcan_data_types/cvra/motor/control/20022.Position.uavcan)
of the board is simple: you send it a position (or speed, or torque), and it
will go there.

The [sensor board](https://github.com/cvra/sensor-board/) (2016) contains three
optical sensors: a laser range finder (10 cm range), a color sensor and an
ambient light sensor. It can be used for object detection around the robot, for
example to check that a game object was correctly handled. It simply publishes
periodic readings on the CAN bus, for anyone interested.

The [IO board](https://github.com/cvra/can-io-board) (2016) has no definite
purpose: by default it is simply 4 digital input / output + 4 PWMs. The original
goal was to control a few industrial sensors or custom electronics. We used it
for many different tasks over the years by reprogramming them to add features.
Two generations of this board exist, with the only difference being the size of
the module.

The [UWB beacon](https://github.com/cvra/uwb-beacon-board) (2017) is still a
work in progress. The long term goal is to provide a system to find the position
of all robots on the playing field by measuring distances with radio (similar to
how GPS receivers work). Antoine is working on them at the moment.

The [actuator board](https://github.com/cvra/actuator-board) (2020) is the
latest addition to the list (2020). Its goal is to be able to control a small
actuator made with RC servos, vacuum pumps and valves. It has vacuum sensors to
check if an object was picked, and has a digital input.

Our [Pi shield](https://github.com/cvra/pi-shield) (2020) allows one to connect
a Raspberry Pi to the bus and to send and receive UAVCAN messages from Linux. It
also allows us to connect a touchscreen placed on the front of the robot.

We have a custom [USB to CAN adapter](https://github.com/cvra/CAN-USB-dongle)
(2015), which has the correct connectors for our robot. It can also optionally
power on the bus (only for a few devices). It is automatically recognized, and
can be set up to be used as a native CAN interface on Linux. Two generations
exist: micro-USB and USB-C. If you are working on the club's projects, you
should probably ask to have one.

## Alternatives considered

When we started looking for what was used as a field bus, we identified big
contenders based on the bandwidth and general availability: I2C[^3], CAN, and
Ethernet-based platforms (IP, ModBus, Ethercat). We removed I2C as it operates
in a master-slave configuration: we wanted the ability for any board to send
messages on the bus. Ethernet-based solutions were the most advanced ones, but
required a lot of circuitry while CAN only required compact single-chip
transceivers.

Originally we had a split master design, where the realtime parts of the master
firmware would be running on a large STM32, while the non-realtime parts would
be written in Python on a PC. The two parts would communicate over Ethernet.
This was extremely complicated and unreliable, so in 2016 we switched to an
architecture with only one master firmware running on STM32. It served us well,
but we were spending a lot of time dealing with low level work, as well as
optimizing to not use too much RAM. This led us to switching back the code to
run on Linux again, but this time including the realtime part as well, with
everything written in C++. You can read more about this switch
[there](https://cvra.ch/robot-software/design/linux-master/).

We experimented with ROS for one year in 2016, using an architecture pretty
similar to the one presented here. The biggest downside of this approach is that
the build tools for ROS are not very nice to use and don't support cross
compilation, which makes building software really slow. The ROS navigation stack
was very CPU hungry, which did not help with our limited CPU resources. It could
certainly be interesting to come back to it now that ROS2 is available. You can
read more about this approach
[here](https://cvra.ch/blog/2016/goldorak-post-mortem).

## Future work

### Communication between robots

The work presented in this article solves the issue of communicating inside the
robot pretty well. However, the rules are moving more and more in the direction
of requiring collaboration between the two robots. In order to do that in an
efficient and safe manner, the two robots need to be able to communicate with
each other.

Several technologies can be used here. Since the two master firmwares are
running on Linux, we can use the normal networking stack to communicate between
the two, either using Wifi, or by reprogramming the UWB boards. Theoretically,
we could even use the two transports in order to provide a redundant link,
however further study is needed.

The higher level protocol is also still an open question. Should we use UDP, in
order to have real time behavior, or TCP to have reliable transmission? Do we
handle errors at the application layer? Do we have something in-between for
reliable ordering of messages (à la Paxos)?

[^1]: The software landscape for FPGA has since changed, and it might be easier
    now thanks to projects like [Yosys](http://www.clifford.at/yosys/) and
    [Chisel](https://www.chisel-lang.org/).
[^2]: Serialization is the process of taking a high level structure and
    translating it to bits on the wire.
[^3]: I2C is typically not considered a fieldbus and was never designed for
    inter-board communication. However it is commonly used in Eurobot due to
    its relative simplicity.
