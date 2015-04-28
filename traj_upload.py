import cvra_rpc.message
import math

from collections import namedtuple

TrajectoryPoint = namedtuple("TrajectoryPoint",
                             ['x', 'y', 'theta', 'speed', 'omega', 'timestamp']
                             )


def prepare_for_sending(traj):
    s = [int(a.timestamp) for a in traj]
    us = [int((a.timestamp - int(a.timestamp))*1e6) for a in traj]

    x = list(zip(s, us, (float(a.x) for a in traj)))
    y = list(zip(s, us, (float(a.y) for a in traj)))
    theta = list(zip(s, us, (float(a.theta) for a in traj)))
    speed = list(zip(s, us, (float(a.speed) for a in traj)))
    omega = list(zip(s, us, (float(a.omega) for a in traj)))

    return [x, y, theta, speed, omega]


def send_traj(host, traj):
    cvra_rpc.message.send(host, 'traj', prepare_for_sending(traj))

def convert_from_molly(traj, start_timestamp):
    res = []
    for t in traj:
        pos, spd, acc, ts = t

        # Equation 8 of tracy's paper
        speed = math.sqrt(spd.pos_x ** 2 + spd.pos_y ** 2)
        if speed > 1e-3:
            omega = (spd.pos_x * acc.pos_y - spd.pos_y * acc.pos_x) / speed ** 2
        else:
            omega = 0

        res.append(TrajectoryPoint(x=pos.pos_x,
                        y=pos.pos_y,
                        theta=math.atan2(spd.pos_y, spd.pos_x),
                        speed=speed,
                        omega=omega,
                        timestamp=start_timestamp + ts
                        ))

    return res


if __name__ == '__main__':
    import time
    now = time.time()
    p1 = TrajectoryPoint(x=1., y=2., theta=3., speed=10., omega=4., timestamp=now+10)
    p2 = TrajectoryPoint(x=1., y=2., theta=3., speed=11., omega=4., timestamp=now+1000)

    traj = [p1, p2]

    send_traj(('192.168.2.20', 20000), traj)
