#!/usr/bin/env python3
"""
Quick and dirty script to send the initial config to a motor board.
"""

import argparse
import uavcan
import yaml


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        "-p",
        help="Serial port or SocketCAN interface",
        required=True)
    parser.add_argument(
        "--config",
        "-c",
        help="YAML config file",
        type=argparse.FileType(),
        required=True)
    parser.add_argument("--dsdl", help="DSDL directory")
    parser.add_argument(
        "--board", "-b", help="Board name to use", required=True)

    return parser.parse_args()


boards_id_by_name = dict()


def node_status_callback(event):
    board = event.transfer.source_node_id
    if board not in boards_id_by_name:
        node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                     _board_info_callback)


def _board_info_callback(event):
    if not event:
        raise RuntimeError("Remote call timeout")

    board = event.transfer.source_node_id
    name = str(event.response.name)
    boards_id_by_name[name] = board


def request_cb(event):
    if not event:
        raise RuntimeError("Remote call timeout")


def main():
    args = parse_args()

    config = yaml.load(args.config)
    config = config['actuator'][args.board]

    if args.dsdl:
        uavcan.load_dsdl(args.dsdl)

    global node
    node = uavcan.make_node(args.port, node_id=127)
    node.add_handler(uavcan.protocol.NodeStatus, node_status_callback)

    req = uavcan.thirdparty.cvra.motor.config.LoadConfiguration.Request()

    req.position_pid.kp = config['control']['position']['kp']
    req.position_pid.ki = config['control']['position']['ki']
    req.position_pid.kd = config['control']['position']['kd']
    req.velocity_pid.kp = config['control']['velocity']['kp']
    req.velocity_pid.ki = config['control']['velocity']['ki']
    req.velocity_pid.kd = config['control']['velocity']['kd']
    req.current_pid.kp = config['control']['current']['kp']
    req.current_pid.ki = config['control']['current']['ki']
    req.current_pid.kd = config['control']['current']['kd']

    req.torque_limit = config['control']['torque_limit']
    req.acceleration_limit = config['control']['acceleration_limit']
    req.low_batt_th = config['control']['low_batt_th']
    req.velocity_limit = config['control']['velocity_limit']

    req.torque_constant = config['motor']['torque_constant']
    req.motor_encoder_steps_per_revolution = config['motor'][
        'motor_encoder_steps_per_revolution']

    try:
        req.second_encoder_steps_per_revolution = config['motor'][
            'second_encoder_steps_per_revolution']
    except KeyError:
        pass

    req.transmission_ratio_p = config['motor']['transmission_ratio_p']
    req.transmission_ratio_q = config['motor']['transmission_ratio_q']
    req.mode = config['motor']['mode']

    while args.board not in boards_id_by_name:
        print("Board not found, waiting...")
        try:
            node.spin(0.2)
        except uavcan.transport.TransferError:
            pass

    board_id = boards_id_by_name[args.board]
    print("Found board, ID={}".format(board_id))

    node.request(req, board_id, request_cb)

    try:
        node.spin(1)
    except uavcan.transport.TransferError:
        pass

    print("Done")


if __name__ == '__main__':
    main()
