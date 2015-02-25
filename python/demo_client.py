from service_call import *
import socket
import msgpack

HOST, PORT = "localhost", 9999

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    data = encode_call("demo", {})

    # Connect to server and send data
    sock.connect((HOST, PORT))
    sock.sendall(data)

    # Receive data from the server and shut down
    received = sock.recv(1024)
    u = msgpack.Unpacker(encoding='ascii')
    u.feed(received)
    print(list(u))
finally:
    sock.close()
