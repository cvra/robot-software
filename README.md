# Microcontroller message bus

## Features

* Runtime declaration of topics
* Many publishers, many subscribers (N to M).
* Subscribers and publishers can be removed without impacting bus.
* Can block waiting for a message.
* Can poll to see if there was an update to the message.
* Topics are atomic.
* Different serialization methods are possible.

## Features that won't be supported

The following features won't be supported, to keep the codebase simple.

* Runtime deletion of topics

