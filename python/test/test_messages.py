from cvra_rpc import message
import msgpack
import threading
import socketserver

import unittest
try:
    from unittest.mock import Mock, patch
except ImportError:
    from mock import Mock


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


class MessageSendTestCase(unittest.TestCase):
    @patch('socket.socket')
    def test_socket_is_closed(self, socket):
        socket.return_value = Mock()
        message.send(('localhost', 1234), 'foo', [1, 2, 3])
        socket.return_value.close.assert_any_call()


class MessageRequestHandlerTestCase(unittest.TestCase):
    def test_factory(self):
        callbacks = {'foo': Mock()}
        RequestHandler = message.create_request_handler(callbacks)
        self.assertEqual(callbacks, RequestHandler.callbacks)

    def test_correct_callback_called(self):
        callbacks = {'foo': Mock(), 'bar': Mock()}

        data = message.encode('bar')
        message.handle_message(data, callbacks)
        callbacks['bar'].assert_any_call([])

    def test_unknown_callbacks_are_ignored(self):
        """
        Checks that unkown callbacks are silently ignored.
        """
        callbacks = {'foo': Mock()}
        data = message.encode('bar')
        message.handle_message(data, callbacks)

    def test_args_forwarded(self):
        callbacks = {'foo': Mock()}
        data = message.encode('foo', [1, 2, 3])

        message.handle_message(data, callbacks)
        callbacks['foo'].assert_any_call([1, 2, 3])

    def test_integration(self):
        """
        Simple integration testing.

        Start one thread for the server, then sends a message to it and checks
        that the callback was correctly installed.
        """
        TARGET = ('localhost', 9999)
        callbacks = {'foo': Mock(), 'bar': Mock()}

        # Creates the server
        RequestHandler = message.create_request_handler(callbacks)
        server = socketserver.UDPServer(TARGET, RequestHandler)

        # Starts the server in another thread
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()

        # Sends the messages
        message.send(TARGET, 'foo', [1, 2, 3])
        message.send(TARGET, 'bar')

        # Terminates the server thread
        server.shutdown()

        # Checks that the callbacks were called
        callbacks['foo'].assert_any_call([1, 2, 3])
        callbacks['bar'].assert_any_call([])
