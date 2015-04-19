import cvra_rpc.service_call
import sys
import yaml

ROBOT = ('192.168.2.20', 20001)
config = yaml.load(open(sys.argv[1]))
print(config)

result = cvra_rpc.service_call.call(ROBOT, 'config_update', [config])
print(result)
