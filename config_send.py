import cvra_rpc.service_call
import yaml
import argparse


def keys_to_str(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {str(k): keys_to_str(v) for k, v in to_convert.items()}


def main():
    parser = argparse.ArgumentParser("Sends the robot config to the master board.")
    parser.add_argument("config", help="YAML file containing robot config.")
    parser.add_argument("master_ip", help="IP address and port of the master board (host:port format).")
    args = parser.parse_args()

    try:
        host, port = args.master_ip.split(":")
    except ValueError:
        host, port = args.master_ip, 20001

    config = yaml.load(open(args.config))
    config = keys_to_str(config)
    print(config)
    result = cvra_rpc.service_call.call((host, port), 'config_update',
                                        [config])
    print(result)

if __name__ == "__main__":
    main()
