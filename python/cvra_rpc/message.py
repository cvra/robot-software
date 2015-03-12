import msgpack

def encode(message_name, args=None):
    """
    Encodes a message, given its name and arguments.
    """
    if args is None:
        args = []

    return msgpack.packb([message_name] + args)

def decode(data):
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(data)
    message = next(u)

    name, data = message[0], message[1:]

    return name, data

