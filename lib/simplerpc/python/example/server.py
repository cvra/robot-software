from cvra_rpc.service_call import *
import socketserver

def demo_callback(params):
    print("I GOT CALLED!")
    return "Hello, client"

if __name__ == "__main__":
    MyTCPHandler = create_request_handler({'demo':demo_callback})

    HOST, PORT = "localhost", 9999
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)

    server.serve_forever()
