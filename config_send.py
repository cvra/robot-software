import cvra_rpc.service_call
import sys
import yaml

def keys_to_str(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {str(k): keys_to_str(v) for k, v in to_convert.items()}



ROBOT = ('192.168.2.20', 20001)
config = yaml.load(open(sys.argv[1]))
config = keys_to_str(config)
print(config)
result = cvra_rpc.service_call.call(ROBOT, 'config_update', [config])
print(result)
