from unittest import TestCase
from collections import namedtuple
from math import sqrt
from math import cos, sin

TrajectoryPoint = namedtuple('TrajectoryPoint', ('timestamp', 'pos', 'omega', 'acc'))

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
        yield TrajectoryPoint(ts, pos, omega, acc)
        ts += dt
        theta += omega * dt


class CircleTrajectoryGenerator(TestCase):
    """
    Checks that a simple robot trajectory generator which runs a circle around
    the origin is working as expected.

    It moves at a given velocity.
    """
    def assertClose(self, a, b, places=2):
        """
        Checks that two points are close.
        """
        self.assertAlmostEqual(0, sqrt((a[0]-b[0])**2+(a[1]-b[1])**2), places=places)

    def test_trajectory_point(self):
        """
        Checks that we can initialize a simple trajectory point.
        """
        p = TrajectoryPoint(timestamp=1.2, pos=(0, 0), omega=1.2, acc=(0, 0))

    def test_generate_trajectory(self):
        """
        Checks that we can generate a full circular trajectory.
        """
        g = generate_circular_traj(r=1.5, omega=2, dt=0.1)
        start = next(g)
        self.assertClose(start.pos, (1.5, 0), places=2)

    def test_trajectory_is_on_circle(self):
        """
        Checks for a few trajectory points that they are on a circle.
        """
        # Check that the first hundred points are at 1.5 m from the origin
        for _, p in zip(range(100), generate_circular_traj(r=1.5, omega=2, dt=0.1)):
            self.assertAlmostEqual(1.5, sqrt(p.pos[0]**2+p.pos[1]**2), places=2)

    def test_timestamps(self):
        """
        Checks that a few timestamps are correct.
        """
        g = generate_circular_traj(r=1.5, omega=2, dt=0.1)
        for i in range(100):
            p = next(g)
            self.assertAlmostEqual(p.timestamp, i*0.1, places=3)

    def test_acc(self):
        """
        Checks that the acceleration appears correct.
        """
        g = generate_circular_traj(r=1.5, omega=2, dt=0.1)
        for i, p in zip(range(100), g):
            self.assertClose(p.acc, (0, 2*2*1.5))
