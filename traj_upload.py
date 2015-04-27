import cvra_rpc.message

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

