# ROS MessagePack bridge
This repository contains the code needed to run a ROS node that will forward the messages he receives to some code running on a microcontroller.
It is an alternative to rosbridge or rosserial, tailored for the needs of the CVRA.

The bridge makes an heavy use of MessagePack and IP networking in order to avoid reinventing the wheel.

# Dependencies
This repository depends on `serial_datagrams` python module, which must be installed.

# Messages
ROS messages are one way communication with no flow control or guarantee of arrival.
They can be thought as a way to stream data.
Messages can be exchanged both ways:
* From microcontroller to PC (sensor measurement, robot position, etc.) or,
* From PC to microcontroller (trajectories for example).

In this bridge, messages are transferred over UDP, with one message per datagram.
The messages are sent to the destination on port 20000 (unprivileged).
The message is encoded as simply its type (a MessagePack string) followed by its arguments (a MessagePack map).

When a message arrives from the microcontroller, the bridge simply converts it to the equivalent ROS message and sends it to other nodes.
When a message arrives from node and is in the list of topic to forward, the bridge encodes it as messagepack and sends it via UDP to the microcontroller.

# Service calls
Service calls are a way to ask the remote end to do something and wait for its answer.
Service calls requests should not be lost, and block until an answer came.
They involve a two way communication between client and server.
Therefore we chose TCP, as it is very useful in that type of situation.

We decided that each service call would involve opening a TCP connection to the server, which would get closed by the server when the call is complete (à la HTTP).
This allows to keep the protocol relatively simple and stateless.

Issuing a request to a service is done as follows:

1. Open a TCP connection to the server on port 20001.
2. The server accepts the connection and forks a handler.
3. The client writes the request, formatted as a message wrapped in a serial datagram (see above).
4. The server reads the message type and parameters and executes whatever action is appropriate.
5. (Optional) the server writes answer data to the client, MessagePack encoded.
6. The server closes the connection.
