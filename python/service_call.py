import msgpack
import socketserver
import socket
import serial_datagrams

def create_request_handler(callbacks_dict):

    class Server(socketserver.BaseRequestHandler):
        callbacks = callbacks_dict

        def handle(self):
            handle_connection(self.callbacks, self.request)

    return Server


def encode_call(method_name, params):
    """
    Encode a call to the given method with the given parameters.
    """
    data = msgpack.packb(method_name) + msgpack.packb(params)
    return serial_datagrams.datagram_encode(data)

def decode_call(data):
    """
    Decodes the given method call.

    Returns a tuple containing the method name and the parameters.
    """
    data = serial_datagrams.datagram_decode(data)
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)

    return tuple(u)

def handle_connection(handlers, socket):
    """
    Correctly handles a connection by looking up the correct callback by name
    in the handlers dict.
    """
    data = socket.recv(1024)
    name, params = decode_call(data)
    retval = handlers[name](params)

    if retval is not None:
        socket.send(msgpack.packb(retval))

def call(adress, method_name, method_args={}):
    """
    Calls the given method on the given adress (a tuple containing hostname and port),
    With the given parameters (dict object).
    """
    connection = socket.create_connection(adress)
    data = encode_call(method_name, method_args)
    connection.sendall(data)

    data = connection.recv(1024)

    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)

    return tuple(u)
