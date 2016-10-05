from cvra_rpc import message
from random import random
import time

TARGET = ('127.0.0.1', 20000)
topic = 'position'

while True:
    data = dict(x=random(), y=random())
    message.send(TARGET, topic, data)
    print("Sent {}: {}".format(topic, data))
    time.sleep(0.2)
