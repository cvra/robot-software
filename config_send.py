#!/usr/bin/env python3
import cvra_rpc.service_call
import yaml
import argparse
import logging
import os.path
import time


def keys_to_str(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {str(k): keys_to_str(v) for k, v in to_convert.items()}


def create_actuator(target, name):
    """
    Creates the actuator to receive the config.
    """
    return cvra_rpc.service_call.call(target, 'actuator_create_driver', [name])


def send_config_file(destination, config_file):
    config = yaml.load(config_file)
    config = keys_to_str(config)
    if "actuator" in config:
        for name in config["actuator"].keys():
            print("Creating actuator {}".format(name))
            create_actuator(destination, name)

    errors = cvra_rpc.service_call.call(destination, 'config_update', [config])
    for key, error in errors:
        logging.warning("Error for key '{}': {}".format(key, error))


def main():
    parser = argparse.ArgumentParser("Sends the robot config to the master board.")
    parser.add_argument("config", help="YAML file containing robot config.")
    parser.add_argument("master_ip", help="IP address and port of the master board (host:port format).")
    parser.add_argument("-w", "--watch",
                        help="Watch config file for changes.",
                        action="store_true")
    args = parser.parse_args()

    try:
        host, port = args.master_ip.split(":")
    except ValueError:
        host, port = args.master_ip, 20001

    send_config_file((host, port), open(args.config))

    if args.watch:
        print("> watching for file changes...")
        old_mtime = os.path.getmtime(args.config)
        while True:
            try:
                try:
                    mtime = os.path.getmtime(args.config)
                # Some editors delete the file before writing to it
                except FileNotFoundError:
                    pass

                if mtime != old_mtime:
                    old_mtime = mtime
                    send_config_file((host, port), open(args.config))

                time.sleep(0.1)
            except KeyboardInterrupt:
                break
            except:
                logging.exception("Unexpected error occured.")

if __name__ == "__main__":
    main()
