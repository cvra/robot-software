def namespace(name):
    return str(name) + '_ns'


class Parameter:
    def __init__(self, name, indent=0):
        self.name = str(name)
        self.indent = indent

    def to_struct(self):
        return '    ' * self.indent + 'parameter_t {};'.format(self.name)


class ParameterNamespace:
    def __init__(self, name, params, indent=0):
        self.name = str(name)
        self.params = params
        self.indent = indent

    def _struct_header(self):
        return [
            'struct {',
            '    parameter_namespace_t {};'.format(namespace(self.name)),
        ]

    def _struct_footer(self):
        return [
            '} ' + self.name + ';',
        ]

    def to_struct(self):
        string = list(map(lambda i: '    ' * self.indent + i, self._struct_header()))

        if self.params is not None:
            for param in self.params:
                string += [param.to_struct()]

        string += list(map(lambda i: '    ' * self.indent + i, self._struct_footer()))

        return '\n'.join(string)


def depth(d, level=1):
    if isinstance(d, dict):
        if d:
            none_to_level = lambda x: level if x is None else x
            dict_as_list = [depth(d[k], level + 1) for k in d]
            return max(map(none_to_level, dict_as_list))
        else:
            return 0


def parse_tree(config, parent='config', level=0):
    if depth(config) == 1:
        return ParameterNamespace(parent, [Parameter(key, level + 1) for key in config], level)
    elif depth(config) > 1:
        children = list()
        for key in config:
            if isinstance(config[key], dict):
                children.append(parse_tree(config[key], key, level + 1))
            else:
                children.append(Parameter(key, level + 1))
        return ParameterNamespace(parent, children, level)
    else:
        return ParameterNamespace(parent, None, level)
