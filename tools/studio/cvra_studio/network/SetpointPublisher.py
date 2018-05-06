import enum
import logging
import time

import uavcan

class ControlTopic(enum.Enum):
    voltage = 'voltage'
    torque = 'torque'
    velocity = 'velocity'
    position = 'position'

    def __str__(self):
        return self.value

    def __call__(self, node_id, value):
        return {
            'voltage': uavcan.thirdparty.cvra.motor.control.Voltage(node_id=node_id, voltage=value),
            'torque': uavcan.thirdparty.cvra.motor.control.Torque(node_id=node_id, torque=value),
            'velocity': uavcan.thirdparty.cvra.motor.control.Velocity(node_id=node_id, velocity=value),
            'position': uavcan.thirdparty.cvra.motor.control.Position(node_id=node_id, position=value),
        }[self.value]

class SetpointPublisher():
    def __init__(self, node, topic, motor, value, period):
        self.node = node
        self.topic = topic
        self.motor = motor
        self.value = value
        self.period = period
        self.handle = node.node.periodic(period, self._publish)
        self.logger = logging.getLogger('SetpointPublisher')
        self.count = 0

    def _publish(self):
        logging.info('Setpoint: {} {} to motor {} at period {}s'.format(self.topic, self.value, self.motor, self.period))
        self.node.node.broadcast(self.topic(node_id=self.motor, value=self.value if self.count % 2 == 0 else - self.value))
        self.count += 1

    def update(self):
        self.handle.remove()
        self.handle = self.node.node.periodic(self.period, self._publish)
