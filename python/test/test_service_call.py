import service_call
from serial_datagrams import *

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
        data = datagram_decode(data)

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

    @patch('socket.create_connection')
    def test_service_call(self, create_connection):
        """
        Check that we correctly send the call.
        """
        create_connection.return_value = Mock()
        create_connection.return_value.recv = Mock(return_value=b'')

        adress = ('127.0.0.1', 20001)
        method_name = 'foo'
        method_params = {'bar': 12}

        expected_data = service_call.encode_call(method_name, method_params)

        service_call.call(adress, method_name, method_params)

        create_connection.assert_any_call(adress)
        create_connection.return_value.sendall.assert_any_call(expected_data)

    @patch('socket.create_connection')
    def test_service_call_answer(self, create_connection):
        """
        Check that the service call is correctly answered.
        """
        create_connection.return_value = Mock()
        adress = ('127.0.0.1', 20001)
        return_data = msgpack.packb(1)+msgpack.packb('bar')

        create_connection.return_value.recv = Mock(return_value=return_data)

        result = service_call.call(adress, 'foo')
        self.assertEqual(result, (1, 'bar'))



class ServiceCallServerTestCase(unittest.TestCase):
    def test_factory(self):
        """
        Checks that the class factory works as expected.
        """
        callbacks = {'foo':Mock()}
        RequestHandler = service_call.create_request_handler(callbacks)
        self.assertEqual(callbacks, RequestHandler.callbacks)



class ServiceCallHandlerTestCase(unittest.TestCase):
    def setUp(self):
        self.handlers = {
                'bar':Mock(return_value=None)
                }

    def test_correct_callback_called(self):
        socket = Mock()
        data = service_call.encode_call('bar', {'x':10})
        socket.recv = Mock(return_value=data)

        service_call.handle_connection(self.handlers, socket)
        self.handlers['bar'].assert_any_call({'x':10})

    def test_callback_writeback(self):
        socket = Mock()
        self.handlers['bar'].return_value = 1
        data = service_call.encode_call('bar', {'x':10})
        socket.recv = Mock(return_value=data)

        service_call.handle_connection(self.handlers, socket)

        expected_data = msgpack.packb(1)
        socket.send.assert_any_call(expected_data)

