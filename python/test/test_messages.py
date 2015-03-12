from cvra_rpc import message
import msgpack

import unittest
try:
    from unittest.mock import *
except ImportError:
    from mock import *


def msgpack_decode(data):
    """
    Helper function that decodes a messagepack object returning strings instead
    of bytes sequence.
    """
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)
    return next(u)

class MessageEncodingTestCase(unittest.TestCase):
    def test_encoding_message_name(self):
        """
        Checks that the message starts with the message type.
        """
        data = message.encode("foo")
        self.assertEqual(msgpack_decode(data), ['foo'])

    def test_encoding_message_parameters(self):
        """
        Checks that we can append parameters.
        """
        data = message.encode("foo", [1, 2, 3])
        self.assertEqual(msgpack_decode(data), ['foo', 1, 2, 3])

class MessageDecodingTestCase(unittest.TestCase):
    def test_decoding_message(self):
        data = message.encode("foo", [1, 2, 3])
        name, args = message.decode(data)

        self.assertEqual('foo', name)
        self.assertEqual([1, 2, 3], args)

