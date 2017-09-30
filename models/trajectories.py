from math import cos, sin
from collections import namedtuple

TrajectoryPoint = namedtuple('TrajectoryPoint', ('timestamp', 'pos', 'theta', 'omega', 'acc'))

def generate_circular_traj(r, omega, dt):
    """
    Generates a series of point on a circle of given radius. The speed and time
    interval between two points must also be given.
    """
    theta = 0.
    ts = 0.
    while True:
        pos = (r * cos(theta), r * sin(theta))
        acc = (0, omega**2 * r)
        yield TrajectoryPoint(ts, pos, theta, omega, acc)
        ts += dt
        theta += omega * dt


