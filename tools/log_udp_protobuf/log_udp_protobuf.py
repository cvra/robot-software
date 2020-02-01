#!/usr/bin/env python3
"""
Logs traffic coming on protobuf from the various topics.
"""

import sys
import os.path
import argparse
import socketserver
import re

from google.protobuf import text_format

import messages


def parse_packet(data):
    messagesize_size = len(messages.MessageSize(bytes=0).SerializeToString())

    header_size = messages.MessageSize()
    header_size.ParseFromString(data[0:messagesize_size])

    cut_point = messagesize_size + header_size.bytes
    header_data, msg_data = data[:cut_point], data[cut_point:]

    header = messages.TopicHeader()
    header.ParseFromString(header_data[messagesize_size:])

    for m in messages.messages:
        if messages.msgid(m) == header.msgid:
            msg = m()
            msg.ParseFromString(msg_data)
            break
    else:
        msg = None

    return header, msg


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", "-p", default=10000, help="Port to listen on (10000)", type=int
    )
    parser.add_argument(
        "--filter", "-f", help="Filter regexp to apply on the topic name"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.filter:
        topic_filter = re.compile(args.filter, flags=re.IGNORECASE)
    else:
        topic_filter = re.compile(".*")

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            data = self.request[0]
            header, msg = parse_packet(data)

            if not topic_filter.search(header.name):
                return

            print("=" * 5)
            print("topic: '{}'".format(header.name))
            print("type: {}".format(msg.DESCRIPTOR.name))
            print("data:")
            print(text_format.MessageToString(msg, indent=2))

    with socketserver.UDPServer(("0.0.0.0", args.port), Handler) as server:
        server.serve_forever()


if __name__ == "__main__":
    main()
