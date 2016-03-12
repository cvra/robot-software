# Microcontroller message bus

## Wanted features

* Runtime declaration of tpoics
* Topic deletion is not supported to simplify pointer lifetime
* One publisher, many subscribers (1 to N).
    N to M would be nice but not necessary.
* Subscribers and publishers can be removed without impacting bus.
* Can block waiting for a message.
* Can poll to see if there was an update to the message.
* Topics are atomic.
* Different serialization methods are possible.
