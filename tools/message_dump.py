import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver
import time

MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)

def msg_cb(todo, msg, args):
    print('receiving:', msg, args)

# stream cvra_rpc -> zmq
RequestHandler = cvra_rpc.message.create_request_handler({}, msg_cb)
msg_server = socketserver.UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
msg_pub_thd = threading.Thread(target=msg_server.serve_forever)
msg_pub_thd.daemon = True
msg_pub_thd.start()

while True:
    time.sleep(1)
