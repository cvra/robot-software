from cvra_rpc.message import *
def foo(args):
    print("foo")
    print(args)

TARGET = ('0.0.0.0', 20000)
callbacks = {'demo': foo}

RequestHandler = create_request_handler(callbacks)
server = socketserver.UDPServer(TARGET, RequestHandler)
server.serve_forever()
