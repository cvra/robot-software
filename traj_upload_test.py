import unittest
from unittest.mock import patch
from traj_upload import *


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


if __name__ == "__main__":
    unittest.main()
