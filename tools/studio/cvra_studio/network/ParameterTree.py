from collections import namedtuple
import logging
import queue

import uavcan


_Parameter = namedtuple('Parameter', ['name', 'value', 'type'])

class Parameter(_Parameter):
    def __repr__(self):
        return str(self.value)

def extract_value(value):
    """
    Given UAVCAN Value object, returns the value and its type
    """
    if   hasattr(value, 'boolean_value'): return (bool(value.boolean_value), bool)
    elif hasattr(value, 'integer_value'): return (int(value.integer_value), int)
    elif hasattr(value, 'real_value'):    return (float(value.real_value), float)
    elif hasattr(value, 'string_value'):  return (str(value.string_value), str)
    else:                                 return (None, None)

def value_to_uavcan(value, value_type):
    """
    Given a value and its type, returns a UAVCAN Value object
    """
    if   value_type == bool:  return uavcan.protocol.param.Value(boolean_value=bool(value))
    elif value_type == int:   return uavcan.protocol.param.Value(integer_value=int(value))
    elif value_type == float: return uavcan.protocol.param.Value(real_value=float(value))
    elif value_type == str:   return uavcan.protocol.param.Value(string_value=str(value))
    else:                     return uavcan.protocol.param.Value()

def parameter_to_yaml(dumper, data):
    """
    Given a YAML dumper and a Parameter data, returns a properly formatted value
    """
    if   data.type == bool:  return dumper.represent_scalar('tag:yaml.org,2002:bool', str(data))
    elif data.type == int:   return dumper.represent_scalar('tag:yaml.org,2002:int', str(data))
    elif data.type == float: return dumper.represent_scalar('tag:yaml.org,2002:float', str(data))
    elif data.type == str:   return dumper.represent_scalar('tag:yaml.org,2002:str', str(data))
    else:                    raise TypeError('Unsupported type {} for parameter'.format(data.type))

class ParameterTree:
    """
    Iterator for accessing a UAVCAN node's parameters
    """
    def __init__(self, node, node_id):
        self.logger = logging.getLogger('ParameterTree')
        self.node = node
        self.node_id = node_id
        self.index = 0
        self.q = queue.Queue()
        self.done = False

    def _request_next(self):
        def callback(event):
            if event:
                value, value_type = extract_value(event.response.value)
                self.q.put(Parameter(name=str(event.response.name), value=value, type=value_type))
                if len(event.response.name) == 0:
                    self.done = True
            else:
                self.logger.warning('Service request has timed out!')
                self.q.put(None)
                self.done = True

        self.node.request(uavcan.protocol.param.GetSet.Request(index=self.index), self.node_id, callback)
        self.index = self.index + 1

    def __iter__(self):
        return self

    def __next__(self):
        self._request_next()
        param = self.q.get(block=True)
        if self.done: raise StopIteration()
        else:         return param
