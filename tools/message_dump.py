import time
import argparse

import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--message", "-m", type=str,
                        help="Message to show, if none then it displays all.",
                        default=None)

    return parser.parse_args()


MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)

def main():
    arguments = parse_arguments()
    print(arguments.message)

    def msg_cb(todo, msg, args):
        if arguments.message is not None:
            if msg == arguments.message:
                print('receiving:', msg, args)
        else:
            print('receiving:', msg, args)

    # stream cvra_rpc -> zmq
    RequestHandler = cvra_rpc.message.create_request_handler({}, msg_cb)
    msg_server = socketserver.UDPServer(MASTER_BOARD_STREAM_ADDR, RequestHandler)
    msg_pub_thd = threading.Thread(target=msg_server.serve_forever)
    msg_pub_thd.daemon = True
    msg_pub_thd.start()

    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
