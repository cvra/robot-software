from cvra_rpc import service_call

class Led:
    """
    Enumeration providing the different LEDs used in the robot.
    """
    Ready = "ready"
    Debug = "debug"
    Error = "error"
    PowerError = "power_error"
    PcError = "pc_error"
    BusError = "bus_error"
    Yellow = ("yellow_1", "yellow_2")
    Green = ("green_1", "green_2")


def set_led(host, led, state):
    """
    Sets the given LED on the given host.

    if led is an iterable but not a string (array for example), every LED in
    the iterable will be set to state.
    """
    if isinstance(led, str):
        res = service_call.call(host, 'led_set', [led, state])
        if res:
            raise RuntimeError("Error setting LED: {}".format(str(res)))
    else:
        for i in led:
            set_led(host, i, state)
