# Design document for UWB protocol v2

## Requirements

* We want to update each robots' positions at 20 Hz.
    - Each distance measurement requires a handshake of 3 packets.
    - We have 4 robots on the playing field.
    - We have 3 fixed beacons.
    Therefore we need to be able to transmit 3 (packets per handshake) x 3 (beacons) x 4 (robots) x 20 (Hz) = 720 packets per seconds.
* We can assume that one beacon is used for synchronizing.
    The election of this beacon can be done through an out of band mechanism, such as flash configuration.
* In addition to range measurements packets, we also need to transmit data between nodes.
    Data packets include the position of the tags, but also any general purpose data packets.
    Data packets are not guaranteed to be delivered exactly once, or in-order (similar to TCP/IP).
* Packets should be authenticated using a secret key.
    This provides an easy way to avoid reacting to bogus packets being sent on the same network.
    It also provides some safety in case another team uses our system, but with non compatible changes.
* The network protocol must not provide automatic configuration.
    It can be nice to have, but is not a requirement.
* The network must not survive to a crash of the synchronizing beacon.
* Collisions should be minimized to maximize link utilization.
* Nodes' local clocks should be assumed to be drifting from each other by a few PPM.
* The protocol should be designed to be able to use hardware MAC filters on the DWM1000.

## Design overview

The protocol is based on Time-Division Multiple Access (TDMA).
Time is divided in frames, and each frame is divided in slots.
During each slot, only one node can transmit, to avoid colisions.

At the beginning of each frame, the Sync Master (SM) sends a packet containing the slot allocation for the next frame.
Each slot is made of (transmitter; receiver) pair for the ranging packets.
Every node must listen to this frame and store the information to know when to speak during next frame.
Then, during each slot, the device allowed to transmit *may* send a message.
Since the DW1000 can be programmed to send a frame at a precise time, we can have very little inter-slot delay without increasing collision.

Data packets are sent at the end of the frame.
Not sure how is the easiest way of doing it.
Giving every node a data slot on every frame means we are loosing a lot of slots when noone wants to talk.
If we stay with little data, it is probably enough to just have one slot at the end, where everybody who wants to send data can do it.
This results in collision in high bandwidth cases though.

## Design parameters

* Number of frames/s: 240
* Numbers of slots/frame: 3

Total: 720 packets/second

Assuming packets of about 1 kilobit, we get:

1000 * (3 (ranging packets) + 1 (allocation packet) + 1 (data packet)) * 240 = 1.2 Mbit/s.

This means we will have to use the highest baudrate of the DW1000 (6.8 Mbps) and with small preamble lengths (64 or 128 bits).
It also means we will not use a very long authentication scheme for packets (32 bits).
