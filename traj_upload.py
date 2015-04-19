from cvra_rpc.message import send
import time

SAMPLE_INTERVAL = 0.1
speed = 0.
while True:
    trajectory = []
    date = int(time.time())
    print(date)
    for i in range(30):
        us = int(1e6 * (date - int(date)))
        s = int(date)
        args = [s, us, speed]
        trajectory.append(args)
        speed += 0.2
        date += SAMPLE_INTERVAL

    send(('192.168.2.20', 20000), 'traj', trajectory)
    time.sleep(0.3)

