# Microcontroller message bus

## Features

* Runtime declaration of topics
* Many publishers, many subscribers (N to M).
* Subscribers and publishers can be removed without impacting bus.
* Can block waiting for a message.
* Can poll to see if there was an update to the message.
* Topics are atomic.
* Different serialization methods are possible.
* Each topic can have a metadata block.
    It can be used to contain function pointers to serialization / deserialization methods for example.
    Metadata do not offer the same atomicity guarantees as the topic data themselves.
* Possibility to register callbacks that are triggered on topic creation.

## Features that won't be supported

The following features won't be supported, to keep the codebase simple.

* Runtime deletion of topics

