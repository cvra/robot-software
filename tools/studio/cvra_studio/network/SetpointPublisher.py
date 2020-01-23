import enum
import logging
import time
import threading

import uavcan


class ControlTopic(enum.Enum):
    voltage = "voltage"
    torque = "torque"
    velocity = "velocity"
    position = "position"

    def __str__(self):
        return self.value

    def __call__(self, node_id, value):
        return {
            "voltage": uavcan.thirdparty.cvra.motor.control.Voltage(
                node_id=node_id, voltage=value
            ),
            "torque": uavcan.thirdparty.cvra.motor.control.Torque(
                node_id=node_id, torque=value
            ),
            "velocity": uavcan.thirdparty.cvra.motor.control.Velocity(
                node_id=node_id, velocity=value
            ),
            "position": uavcan.thirdparty.cvra.motor.control.Position(
                node_id=node_id, position=value
            ),
        }[self.value]


class SetpointPublisher:
    def __init__(self, node, topic, motor, value_min, value_max, period):
        self.node = node
        self.topic = topic
        self.motor = motor
        self.value_min = value_min
        self.value_max = value_max
        self.period = period
        self.lock = threading.RLock()
        self.handle = node.node.periodic(0.01, self._publish)
        self.logger = logging.getLogger("SetpointPublisher")
        threading.Thread(target=self._update).start()

    def _publish(self):
        with self.lock:
            logging.info(
                "Setpoint: {} {} to motor {} at period {}s".format(
                    self.topic, self.value, self.motor, self.period
                )
            )
            self.node.node.broadcast(self.topic(node_id=self.motor, value=self.value))

    def _update(self):
        while True:
            with self.lock:
                self.value = self.value_min
            time.sleep(self.period)
            with self.lock:
                self.value = self.value_max
            time.sleep(self.period)

    def update(self):
        self.handle.remove()
        self.handle = self.node.node.periodic(0.01, self._publish)
