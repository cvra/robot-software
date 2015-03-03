from service_call import *
import socket
import msgpack

HOST, PORT = "localhost", 9999

result = call((HOST, PORT), 'demo')
print(result)
