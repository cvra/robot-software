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
        TrajectoryPoint(x=0., y=0., theta=0., speed=1., omega=0.,
                        timestamp=1.4)

    def test_can_prepare_trajectory(self):
        traj = [TrajectoryPoint(x=1., y=2., theta=3., speed=4.,
                                omega=5., timestamp=float(i))
                for i in range(10)]

        traj_transformed = prepare_for_sending(traj)

        self.assertEqual(traj_transformed[0], [(i, 0, 1.) for i in range(10)])
        self.assertEqual(traj_transformed[1], [(i, 0, 2.) for i in range(10)])
        self.assertEqual(traj_transformed[2], [(i, 0, 3.) for i in range(10)])
        self.assertEqual(traj_transformed[3], [(i, 0, 4.) for i in range(10)])
        self.assertEqual(traj_transformed[4], [(i, 0, 5.) for i in range(10)])

    def test_can_send(self):
        traj = [TrajectoryPoint(x=1., y=2., theta=3., speed=4.,
                                omega=5., timestamp=float(i))
                for i in range(10)]

        preprocessed_traj = prepare_for_sending(traj)

        with patch('cvra_rpc.message.send') as s:
            host = ('foo', 2000)
            send_traj(host, traj)
            s.assert_any_call(host, 'traj', preprocessed_traj)


class MollyAdapterTestCase(unittest.TestCase):
    def test_can_transform_position_timestamp(self):
        traj = [(Vec2D(1., 2.), Vec2D(), Vec2D(), 0.3)]
        traj = convert_from_molly(traj, start_timestamp=10.)

        t = traj[0]

        self.assertEqual(t.x, 1.)
        self.assertEqual(t.y, 2.)
        self.assertAlmostEqual(t.timestamp, 10.3, 5)

    def test_can_transform_speed(self):
        traj = [(Vec2D(), Vec2D(0., 1.), Vec2D(), 0.3)]
        t = convert_from_molly(traj, start_timestamp=10.)[0]
        self.assertAlmostEqual(t.speed, 1., 5)
        self.assertAlmostEqual(t.theta, pi / 2, 3)

    def test_can_transform_acceleration(self):
        # Circular trajectory, a = v ** 2 / r, v = 2, r = 1
        # -> a = 4 m / s^2, omega = 2 rad / s
        traj = [(Vec2D(), Vec2D(2., 0.), Vec2D(0., 4), 0.3)]
        t = convert_from_molly(traj, start_timestamp=10.)[0]
        self.assertAlmostEqual(t.omega, 2., 3)


if __name__ == "__main__":
    unittest.main()
