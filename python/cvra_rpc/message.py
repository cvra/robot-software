import msgpack
try:
    import socketserver
except ImportError:
    import SocketServer as socketserver
import socket


def encode(message_name, args=None):
    """
    Encodes a message, given its name and arguments.
    """
    if args is None:
        args = []

    return msgpack.packb([message_name] + args)


def decode(data):
    """
    Decodes the given datagram to a tuple (name, args).
    """
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)
    message = next(u)

    name, data = message[0], message[1:]

    return name, data


def create_request_handler(callbacks_dict):
    """
    Returns a class derived from BaseRequestHandler that can be used with UDP
    socket server.
    """
    class Server(socketserver.BaseRequestHandler):
        callbacks = callbacks_dict

        def handle(self):
            handle_message(self.request[0], self.callbacks)

    return Server


def send(target, name, args=None):
    """
    Sends a message with the given name and the given args.
    target is a tuple in the (hostname, port).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = encode(name, args)
    sock.sendto(data, target)
    sock.close()


def handle_message(data, callbacks):
    """
    Handles a single datagram and calls the correct callback.
    """
    name, args = decode(data)
    callbacks[name](args)
