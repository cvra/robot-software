import msgpack


def encode_call(method_name, params):
    """
    Encode a call to the given method with the given parameters.
    """
    return msgpack.packb(method_name) + msgpack.packb(params)

def decode_call(data):
    """
    Decodes the given method call.

    Returns a tuple containing the method name and the parameters.
    """
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)

    return tuple(u)

def handle_connection(handlers, socket):
    """
    Correctly handles a connection by looking up the correct callback by name in the handlers dict.
    """
    data = socket.recv(1024)
    name, params = decode_call(data)
    handlers[name](params)


