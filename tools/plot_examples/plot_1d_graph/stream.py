from cvra_rpc import message
from random import random
import time
import math

TARGET = ('127.0.0.1', 20000)
topic = 'var'

start = time.time()

while True:
    data = dict(x=math.sin(time.time() - start))
    message.send(TARGET, topic, data)
    print("Sent {}: {}".format(topic, data))
    time.sleep(0.05)
