import service_call

import unittest
try:
    from unittest.mock import *
except ImportError:
    from mock import *

import msgpack

class ServiceCallEncodingTestCase(unittest.TestCase):
    def test_encoding_method_name(self):
        """
        Checks that encoding method works as expected.
        """
        data = service_call.encode_call("foo", {'bar':42})

        u = msgpack.Unpacker(encoding='ascii')
        u.feed(data)
        command = list(u)

        self.assertEqual(command[0], "foo")
        self.assertEqual(command[1]['bar'], 42)

    def test_decoding_method(self):
        """
        Checks that we can decode a method.
        """
        data = service_call.encode_call("foo", {'bar':42})
        name, params = service_call.decode_call(data)

        self.assertEqual(name, "foo")
        self.assertDictEqual({'bar':42}, params)

class ServiceCallHandlerTestCase(unittest.TestCase):
    def setUp(self):
        self.handlers = {
                'foo':Mock(),
                'bar':Mock()
                }

    def test_callback_called(self):
        socket = Mock()
        socket.recv = Mock(return_value=service_call.encode_call('foo', {}))

        service_call.handle_connection(self.handlers, socket)
        self.handlers['foo'].assert_any_call({})

    def test_correct_callback_called(self):
        socket = Mock()
        data = service_call.encode_call('bar', {'x':10})
        socket.recv = Mock(return_value=data)

        service_call.handle_connection(self.handlers, socket)
        self.handlers['bar'].assert_any_call({'x':10})



