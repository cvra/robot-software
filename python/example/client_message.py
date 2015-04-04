from cvra_rpc.message import *

from sys import argv

state = int(argv[1])

send(('192.168.2.20', 20000), 'test', [bool(state)])
