class Parameter:
    def __init__(self, name, value, parents, indent=0):
        self.name = str(name)
        self.indent = indent
        self.parents = parents
        self.value = value
        self.var = self.name.replace('-', '_')

    def to_struct(self):
        indent = '    ' * self.indent
        s = indent + 'parameter_t {var};'

        if isinstance(self.value, str):
            s += '\n' + indent + 'char {var}_buffer[16];'

        return s.format(var=self.var)

    def to_init_code(self):
        s = '    '
        if isinstance(self.value, bool):
            s += 'parameter_boolean_declare(&{parent}.{var}, &{parent}.ns, "{name}");'
        elif isinstance(self.value, int):
            s += 'parameter_integer_declare(&{parent}.{var}, &{parent}.ns, "{name}");'
        elif isinstance(self.value, float):
            s += 'parameter_scalar_declare(&{parent}.{var}, &{parent}.ns, "{name}");'
        elif isinstance(self.value, str):
            s += 'parameter_string_declare(&{parent}.{var}, &{parent}.ns, "{name}", {parent}.{name}_buffer, sizeof({parent}.{name}_buffer));'
        else:
            raise TypeError('[Parameter] Unsupported type: {}'.format(type(self.value)))

        return s.format(var=self.var, name=self.name, parent='.'.join(self.parents))


class ParameterNamespace:
    def __init__(self, name, params, parents=[], indent=0):
        self.name = str(name)
        self.params = params
        self.indent = indent
        self.parents = parents
        self.var = self.name.replace('-', '_')

    def _struct_header(self):
        return [
            'struct {',
            '    parameter_namespace_t ns;',
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

    def _is_root(self):
        return len(self.parents) == 0

    def _code_parent_ns(self):
        return '&{}.ns'.format('.'.join(self.parents))

    def _code_name(self):
        return '"{}"'.format(self.name)

    def _code_ns(self):
        return '&{}.ns'.format('.'.join(self.parents + [self.var]))

    def to_init_code(self, function_name='config_init'):
        if self._is_root():
            string = [
                'void {}(void)'.format(function_name),
                '{',
                '    parameter_namespace_declare({struct}, NULL, NULL);'.format(struct=self._code_ns()),
            ]
        else:
            string = [
                '    parameter_namespace_declare({struct}, {parent}, {name});'.format(
                    struct=self._code_ns(), parent=self._code_parent_ns(), name=self._code_name())
            ]

        if self.params is not None:
            for param in self.params:
                string += [param.to_init_code()]

        if self._is_root():
            string += ['}']

        return '\n'.join(string)


def depth(d, level=1):
    if isinstance(d, dict):
        if d:
            none_to_level = lambda x: level if x is None else x
            dict_as_list = [depth(d[k], level + 1) for k in d]
            return max(map(none_to_level, dict_as_list))
        else:
            return 0


def parse_tree(config, name='config', level=0, parents=list()):
    if depth(config) == 1:
        return ParameterNamespace(name, [Parameter(key, config[key], parents + [name], level + 1) for key in config], parents, level)
    elif depth(config) > 1:
        children = list()
        for k, v in config.items():
            if isinstance(v, dict):
                children.append(parse_tree(v, k, level + 1, parents + [name]))
            else:
                children.append(Parameter(k, v, parents + [name], level + 1))
        return ParameterNamespace(name, children, parents, level)
    else:
        return ParameterNamespace(name, None, parents, level)
