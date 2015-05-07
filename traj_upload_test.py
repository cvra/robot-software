import unittest
try:
    from unittest.mock import patch
except ImportError:
    from mock import patch

from traj_upload import *
from molly.Vec2D import Vec2D
from math import pi


class TrajectoryTestCase(unittest.TestCase):
    def test_can_create_traj_point(self):
        TrajectoryPoint(x=1., y=2., theta=3., speed=4., omega=5.)

    def test_can_create_traj(self):
        Trajectory(start_time=12., sampling_time=0.1,
                   points=[TrajectoryPoint(x=1., y=2., theta=3., speed=4.,
                                           omega=5.)])

    def test_can_prepare_trajectory(self):
        points = [TrajectoryPoint(x=1., y=2., theta=3., speed=4.,
                                  omega=float(i)) for i in range(10)]

        traj = Trajectory(start_time=10.5, sampling_time=0.1, points=points)

        traj = prepare_for_sending(traj)

        self.assertEqual(traj[0], 10)  # Starting s
        self.assertEqual(traj[1], 500000)  # Starting us
        self.assertEqual(traj[2], 100000)  # Sampling time us

        for i, point in enumerate(traj[3]):
            self.assertEqual(point, [1., 2., 3., 4., float(i)])

    def test_can_send(self):
        points = [TrajectoryPoint(x=1., y=2., theta=3., speed=4.,
                                  omega=5.) for i in range(10)]

        traj = Trajectory(start_time=10.5, sampling_time=0.1, points=points)

        preprocessed_traj = prepare_for_sending(traj)

        with patch('cvra_rpc.message.send') as s:
            host = ('foo', 2000)
            send_traj(host, traj)
            s.assert_any_call(host, 'traj', preprocessed_traj)


class MollyAdapterTestCase(unittest.TestCase):
    def test_can_transform_position_timestamp(self):
        traj = [(Vec2D(1., 2.), Vec2D(), Vec2D(), 0.3)]
        traj = convert_from_molly(traj, start_time=10., sampling_time=0.1)

        self.assertEqual(traj.start_time, 10.)
        self.assertEqual(traj.sampling_time, .1)

        t = traj.points[0]

        self.assertEqual(t.x, 1.)
        self.assertEqual(t.y, 2.)

    def test_can_transform_speed(self):
        traj = [(Vec2D(), Vec2D(0., 1.), Vec2D(), 0.3)]
        t = convert_from_molly(traj, start_time=10., sampling_time=0.1)
        t = t.points[0]

        self.assertAlmostEqual(t.speed, 1., 5)
        self.assertAlmostEqual(t.theta, pi / 2, 3)

    def test_can_transform_acceleration(self):
        # Circular trajectory, a = v ** 2 / r, v = 2, r = 1
        # -> a = 4 m / s^2, omega = 2 rad / s
        traj = [(Vec2D(), Vec2D(2., 0.), Vec2D(0., 4), 0.3)]
        t = convert_from_molly(traj, start_time=10., sampling_time=0.1)
        t = t.points[0]
        self.assertAlmostEqual(t.omega, 2., 3)


class TrajectorySlicerTestCase(unittest.TestCase):
    def test_can_slice(self):
        points = [TrajectoryPoint(x=i, y=2., theta=3., speed=4.,
                                  omega=5.) for i in range(96)]

        traj = Trajectory(start_time=0., sampling_time=0.1, points=points)
        sliced = list(trajectory_slice(traj, 10))
        self.assertEqual(10, len(sliced))

        self.assertEqual(10, len(sliced[0].points))
        self.assertEqual(10., sliced[1].points[0].x)

        self.assertEqual(6, len(sliced[-1].points))
        self.assertEqual(95, sliced[-1].points[5].x)


if __name__ == "__main__":
    unittest.main()
