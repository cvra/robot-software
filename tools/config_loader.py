#!/usr/bin/env python3

import uavcan
import argparse
import logging
import yaml
import collections

# https://gist.github.com/angstwad/bf22d1822c38a92ec0a9
def dict_merge(dct, merge_dct):
    """ Recursive dict merge. Inspired by :meth:``dict.update()``, instead of
    updating only top-level keys, dict_merge recurses down into dicts nested
    to an arbitrary depth, updating keys. The ``merge_dct`` is merged into
    ``dct``.
    :param dct: dict onto which the merge is executed
    :param merge_dct: dct merged into dct
    :return: None
    """
    for k, v in merge_dct.items():
        if (k in dct and isinstance(dct[k], dict)
                and isinstance(merge_dct[k], collections.Mapping)):
            dict_merge(dct[k], merge_dct[k])
        else:
            dct[k] = merge_dct[k]


def config_send(node, config, can_id):
    # create request
    req = uavcan.thirdparty.cvra.motor.config.LoadConfiguration.Request()

    # fill config message
    req.position_pid.kp = config['control']['position']['kp']
    req.position_pid.ki = config['control']['position']['ki']
    req.position_pid.kd = config['control']['position']['kd']
    req.position_pid.ilimit = config['control']['position']['ilimit']

    req.velocity_pid.kp = config['control']['velocity']['kp']
    req.velocity_pid.ki = config['control']['velocity']['ki']
    req.velocity_pid.kd = config['control']['velocity']['kd']
    req.velocity_pid.ilimit = config['control']['velocity']['ilimit']

    req.current_pid.kp = config['control']['current']['kp']
    req.current_pid.ki = config['control']['current']['ki']
    req.current_pid.kd = config['control']['current']['kd']
    req.current_pid.ilimit = config['control']['current']['ilimit']

    req.torque_limit = config['control']['torque_limit']
    req.velocity_limit = config['control']['velocity_limit']
    req.acceleration_limit = config['control']['acceleration_limit']
    req.low_batt_th = config['control']['low_batt_th']

    req.thermal_capacity = config['thermal']['capacity']
    req.thermal_resistance = config['thermal']['resistance']
    req.thermal_current_gain = config['thermal']['current_gain']
    req.max_temperature = config['thermal']['max_temperature']

    req.torque_constant = config['motor']['torque_constant']
    req.transmission_ratio_p = config['motor']['transmission_ratio_p']
    req.transmission_ratio_q = config['motor']['transmission_ratio_q']
    req.motor_encoder_steps_per_revolution = config['motor']['motor_encoder_steps_per_revolution']
    req.second_encoder_steps_per_revolution = config['motor']['second_encoder_steps_per_revolution']
    req.potentiometer_gain = config['motor']['potentiometer_gain']
    req.mode = config['motor']['mode']

    # send request
    node.request(req, can_id, lambda event: print('Load configuration OK!'))


def config_fill_missing(update):
    # default values
    config = {
    'control': {
        'position': {'kp': 0, 'ki': 0, 'kd': 0, 'ilimit': 0},
        'velocity': {'kp': 0, 'ki': 0, 'kd': 0, 'ilimit': 0},
        'current': {'kp': 0, 'ki': 0, 'kd': 0, 'ilimit': 0},
        'torque_limit': 0,
        'velocity_limit': 0,
        'acceleration_limit': 0,
        'low_batt_th': 12},
    'thermal':{
        'capacity': 0.,
        'resistance': 0.,
        'current_gain': 0.,
        'max_temperature': 0.},
    'motor': {
        'torque_constant': 1,
        'transmission_ratio_p': 1,
        'transmission_ratio_q': 1,
        'motor_encoder_steps_per_revolution': 4096,
        'second_encoder_steps_per_revolution': 4096,
        'potentiometer_gain': 1,
        'mode': 4}
    }
    dict_merge(config, update)
    return config

class NodeDiscovery:
    def __init__(self, node, node_name):
        self.node_name = node_name
        self.node_id = None
        self.node = node
        self.node.add_handler(uavcan.protocol.NodeStatus,
                self._node_status_callback)

    def _board_info_callback(self, event):
        board = event.transfer.source_node_id
        name = str(event.response.name)
        if name == self.node_name:
            self.node_id = board

    def _node_status_callback(self, event):
        board = event.transfer.source_node_id
        self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                self._board_info_callback)

    def get_id(self):
        return self.node_id

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="SocketCAN interface (e.g. can0) or SLCAN serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("config", help="YAML file containing robot config.")
    parser.add_argument("board", help="Board name")
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)

    return parser.parse_args()

def main():
    args = parse_args()

    with open(args.config) as file:
        config = yaml.load(file)

    config = config['actuator'][args.board]
    config = config_fill_missing(config)

    # start UAVCAN node
    uavcan.load_dsdl(args.dsdl)
    node = uavcan.make_node(args.port, node_id=127)

    # set up board discovery
    disc = NodeDiscovery(node, args.board)

    while disc.get_id() is None:
        node.spin(0.1)

    print('board "{}" has ID {}'.format(args.board, disc.get_id()))

    print('Load config...')
    config_send(node, config, disc.get_id())

    node.spin(0.1)

if __name__ == '__main__':
    main()
