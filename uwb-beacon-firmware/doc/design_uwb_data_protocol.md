---
title: Design of a data protocol for CVRA's UWB network
author: "@antoinealb"
---

# Problem description

Our robots embed a wireless system based on IEEE 802.15.4 standards.
Its main purpose was positioning; the wireless modules used (DW1000) allow for high precision time of flight measurements, which can be used in a beacon based position calculation.
However those modules can also be used for data transmission with the following properties:

* Robust: those modules uses Ultra Wide Band technology, meaning they have more energy and are less susceptible to interference than Wifi.
    In addition, they can be configured to use other frequencies than traditional radios, making them stronger again.
* Reasonable bitrate: The manufacturer claims that they can reach 6.8 Mbps.
    However, this relies on advanced hardware features that our current implementation is not using.
* Large MTU: Through the use of non standard extensions to the 802.15.4 protocol, DW1000's frames can carry up to 1023 bytes of payload.
* Hardware frame filtering: The hardware can run filters on MAC addresses to avoid unnecessary processing by the application processor.
    This is not used in our current design, but could be added as a future enhancement.

# Proposed design

We propose to use IPv4 packets and pack them in a UWB frame.
No MAC address filtering is enabled; frames are always sent as broadcast, and filtering is done through the IP layer.
The design follows the end-to-end principle: the UWB beacon boards are not running an IP or TCP layer: they received serialized frames on their CAN interface, and transmit them over UWB.
Incoming UWB frames are serialized on the CAN interface.
This allows the master board to do the heavy lifting of networking.

## Why IP ?

Typical solutions for this type of network use custom made point-to-point protocols.
While this choice is more efficient, we believe that IP would be a good fit, for the following reasons:

* IP is lindy: IP was there, IP is there and IP will be there.
    It is currently the lingua franca of computer networks and is very well understood.
* IP comes with a lot of debug tools for free.
    Things like tcpdump, Wireshark, or simply ping are invaluable when diagnosing network issues.
* Lots of higher level protocols build on top of IP.
    This means that if we need to provide a reliable ordered protocol for example, we can use TCP for this purpose.
    If we need a naming service, then DNS is there for us.
    However, if we just need a small connection multiplexing protocol, we can stick with UDP.
* IPv4 fits on the UWB packet size.
    More specifically, IPv4 requires packets of at least 576 bytes, while IPv6 requires 1280 bytes per packet.
    Note that smaller packets are possible and those values are just those from the standard.
* IP was designed to be routed; while this is not useful right now, it could become interesting if we go back to having Linux PCs in the robots.
* IP has a relatively low overhead: about 20 bytes per packet.
    UDP adds about 8 bytes per packet.
    This means that on a full size packet of 1023 bytes, the application would be able to use 995 bytes, or 97% of the packet.

## Reasons not to use IP

* Complexity.
    Running a homebrew protocol is much simpler, but less flexible.
* Memory footprint.
    A complete TCP/IP stack is expensive to run, both in term of flash and main memory.
    However we will probably only use UDP, meaning TCP can be removed from the build.
    On lwip this means up to 50% of (source) code size reduction.
* Efficiency.
    It is definitely possible to have a lower overhead than IP with a custom protocol.
* Duplication of network addresses.
    Each device will then have one MAC address and one IP address.
    On Ethernet this is typically solved using the Adress Resolution Protocol (ARP), which provides a mapping from IP to MAC address.
    We will use broadcast frames as a first version, and ARP support can be added later.

# Implementation

## Robot

On the robot, the master board will be (already is) running the lwip IP stack.
A custom network interface will be written, which sends IP packets over UAVCAN messages.
For this purpose, a new UAVCAN message type called `NetworkData` will be provided.

The UWB beacons will be listening for UAVCAN data packets and send them over UWB.
When they receive a UWB frame, they will forward them to the UAVCAN bus.

Note that this separates the radio layer concerns from the IP layer concerns: It should be possible to run an IP network between two master boards by connecting their CAN interfaces directly.
This can be useful for testing.

## PC

Another type of network participants we want to have will be PCs.
They are used for two purposes:

1. At least one PC will be used for detecting playing field elements.
    As it will be on the side of the game area and not on the robots, it needs to send messages wirelessly.
    This machine will be running Linux, most likely Ubuntu.
2. Dev laptops will be connected to the robot for testing & debugging.
    Here we expect macOS and Linux (various) machines; Windows is considered out of scope.

The PC interface will be provided by a userland implementation of the UAVCAN serializer and a TUN interface.
The script will then provide the serialization mechanism.
