from master_board import RPC_PORT, MSG_PORT
import unittest


class DefaultTestCase(unittest.TestCase):
    def test_rpc_port(self):
        self.assertEqual(RPC_PORT, 20001)
        self.assertEqual(MSG_PORT, 20000)
