from collections import namedtuple
import queue

import uavcan


Parameter = namedtuple('Parameter', ['name', 'value'])

def extract_value(value):
    if   hasattr(value, 'boolean_value'): return bool(value.boolean_value)
    elif hasattr(value, 'integer_value'): return int(value.integer_value)
    elif hasattr(value, 'real_value'):    return float(value.real_value)
    elif hasattr(value, 'string_value'):  return str(value.string_value)
    else:                                 return None

class ParameterTree:
    def __init__(self, node, node_id):
        self.node = node
        self.node_id = node_id
        self.index = 0
        self.q = queue.Queue()
        self.done = False

    def _request_next(self):
        def callback(event):
            if event:
                self.q.put(Parameter(name=str(event.response.name), value=extract_value(event.response.value)))
                if len(event.response.name) == 0:
                    self.done = True
            else:
                raise Exception('Service request has timed out!')

        self.node.request(uavcan.protocol.param.GetSet.Request(index=self.index), self.node_id, callback)
        self.index = self.index + 1

    def __iter__(self):
        return self

    def __next__(self):
        self._request_next()
        param = self.q.get(block=True)
        if self.done: raise StopIteration()
        else:         return param
