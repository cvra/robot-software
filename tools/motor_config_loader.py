#!/usr/bin/env python3
"""
Loads all parameters from a YAML config to the motor boards.
"""
import argparse
import uavcan
import threading
import logging
import progressbar
import yaml
import time

OPCODE_SAVE = 0

def parameters_from_config(config, prefix=""):
    """
    Generates a serie of (url, value) parameters for the given config tree.
    """
    for name, value in config.items():
        name = prefix + "/" + name
        if isinstance(value, dict):
            yield from parameters_from_config(value, prefix=name)
        else:
            yield (name, value)


class ConfigUploader:
    """
    This class is used to send a whole config to the motor boards.
    """
    boards_id_by_name = dict()
    shouldQuit = False

    def __init__(self, interface):
        self.node_lock = threading.RLock()
        self.rpc_answered = threading.Semaphore(value=0)
        self.node = uavcan.make_node(interface, node_id=127)
        threading.Thread(target=self._uavcan_thread).start()
        self.node.add_handler(uavcan.protocol.NodeStatus,
                              self._node_status_callback)

    def _uavcan_thread(self):
        while not self.shouldQuit:
            with self.node_lock:
                self.node.spin(0.1)
            time.sleep(0.01)

    def _node_status_callback(self, event):
        board = event.transfer.source_node_id
        logging.debug("Got a node status from {}".format(board))
        if board not in self.boards_id_by_name.values():
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), board,
                              self._board_info_callback)

    def _board_info_callback(self, event):
        if not event:
            raise RuntimeError("Remote call timeout")

        board = event.transfer.source_node_id
        name = str(event.response.name)
        logging.debug("Discovered board {}".format(name))
        self.boards_id_by_name[name] = board

    def send_parameter(self, name, value, board_name):
        with self.node_lock:
            request = uavcan.protocol.param.GetSet.Request(name=name)
            # TODO other types
            if type(value) == bool:
                request.value.boolean_value = value
            elif type(value) == int:
                request.value.integer_value = value
            else:
                request.value.real_value = value

            self.node.request(request, self.boards_id_by_name[board_name],
                              self._send_parameter_callback)

        # Wait for an answer
        self.rpc_answered.acquire()

    def _send_parameter_callback(self, event):
        if not event:
            raise RuntimeError("RPC Timeout")

        if not event.response.name:
            raise RuntimeError("No such parameter!")

        self.rpc_answered.release()

    def save_parameters(self, board_name):
        with self.node_lock:
            request = uavcan.protocol.param.ExecuteOpcode.Request()
            request.opcode = OPCODE_SAVE
            self.node.request(request, self.boards_id_by_name[board_name],
                              self._save_parameters_callback)

    def _save_parameters_callback(self, event):
        if not event:
            raise RuntimError("RPC Timeout")

        if not event.response.ok:
            raise RuntimeError("Could not save config on board {}".format(
                event.transfer.source_node_id))

        self.rpc_answered.release()

    def upload_config(self, configs):
        """
        This method uploads the whole config tree given in parameter to the
        corresponding motor boards.
        """
        # Wait for all boards to be found
        print("Finding boards...")
        pbar = progressbar.ProgressBar(maxval=len(configs)).start()
        found_boards = set(self.boards_id_by_name) & set(configs)
        while len(configs) > len(found_boards):
            time.sleep(0.1)
            pbar.update(len(found_boards))
            found_boards = set(self.boards_id_by_name) & set(configs)
        pbar.finish()

        print("Sending parameters...")
        total_param_count = sum(
            len(list(parameters_from_config(c))) for c in configs.values())
        pbar = progressbar.ProgressBar(maxval=total_param_count).start()
        current_param_count = 0
        for board, config in configs.items():
            for param, value in parameters_from_config(config):
                logging.debug("Sending {} = {} to {}".format(
                    param, value, board))
                self.send_parameter(param, value, board)
                current_param_count += 1
                pbar.update(current_param_count)
        pbar.finish()

        print("Saving parameters to flash...")
        pbar = progressbar.ProgressBar(maxval=len(configs)).start()
        for i, board in enumerate(configs.keys()):
            pbar.update(i + 1)
            self.save_parameters(board)
        pbar.finish()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface", help="Serial port or SocketCAN interface")
    parser.add_argument(
        "config_file", help="Config file in yml", type=argparse.FileType())
    parser.add_argument("--verbose", "-v", action='store_true')
    parser.add_argument("--dsdl", "-d", help="DSDL path", required=True)
    parser.add_argument('-b','--board', action='append', help='Boards to consider, defaults to all')

    return parser.parse_args()


def main():
    args = parse_args()
    config = yaml.load(args.config_file)

    if args.verbose:
        level = logging.DEBUG
    else:
        level = logging.WARNING

    logging.basicConfig(level=level)

    uavcan.load_dsdl(args.dsdl)

    uploader = ConfigUploader(args.interface)

    if args.board is not None:
        configs = {board: config['actuator'][board] for board in args.board}
    else:
        configs = config['actuator']

    uploader.upload_config(configs)
    uploader.shouldQuit = True


if __name__ == '__main__':
    main()
