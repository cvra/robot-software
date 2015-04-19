import cvra_rpc.service_call
import sys

ROBOT = ('192.168.2.20', 20001)
# config = {'master': {'foo': float(sys.argv[1])}}
config = {'master': {'foo': 'asd'}}
result = cvra_rpc.service_call.call(ROBOT, 'config_update', [config])
print(result)
