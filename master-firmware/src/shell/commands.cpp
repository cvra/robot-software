#include "commands.h"
#include <parameter/parameter.h>
#include <absl/strings/str_format.h>
#include "main.h"

static void command_reboot(std::ostream& stream, int /* argc */, const char** /*argv*/)
{
    stream << "Exiting now, the restart will be handled by systemd" << std::endl;
    exit(1);
}

static void tree_indent(std::ostream& out, int indent)
{
    for (int i = 0; i < indent; i++) {
        out << "  ";
    }

    std::flush(out);
}

static void show_config_tree(std::ostream& out, parameter_namespace_t* ns, int indent)
{
    parameter_t* p;

    tree_indent(out, indent);

    if (ns->id) {
        out << ns->id << std::endl;
    }

    for (p = ns->parameter_list; p != NULL; p = p->next) {
        tree_indent(out, indent + 1);
        if (parameter_defined(p)) {
            switch (p->type) {
                case _PARAM_TYPE_SCALAR:
                    out << absl::StrFormat("%s: %f", p->id, parameter_scalar_get(p)) << std::endl;
                    break;

                case _PARAM_TYPE_INTEGER:
                    out << absl::StrFormat("%s: %d", p->id, parameter_integer_get(p)) << std::endl;
                    break;

                case _PARAM_TYPE_BOOLEAN:
                    if (parameter_boolean_get(p)) {
                        out << absl::StrFormat("%s: true", p->id) << std::endl;
                    } else {
                        out << absl::StrFormat("%s: false", p->id) << std::endl;
                    }
                    break;

                case _PARAM_TYPE_STRING: {
                    static char buf[50];
                    parameter_string_get(p, buf, sizeof(buf));
                    out << absl::StrFormat("%s: \"%s\"", p->id, buf) << std::endl;
                    break;
                }

                default:
                    out << absl::StrFormat("%s: unknown type %d", p->id, p->type) << std::endl;
                    break;
            }
        } else {
            out << absl::StrFormat("%s: not set", p->id) << std::endl;
        }
    }

    if (ns->subspaces) {
        show_config_tree(out, ns->subspaces, indent + 1);
    }

    if (ns->next) {
        show_config_tree(out, ns->next, indent);
    }
}

static void command_config_tree(std::ostream& out, int argc, const char** argv)
{
    parameter_namespace_t* ns;
    if (argc < 2) {
        ns = &global_config;
    } else {
        ns = parameter_namespace_find(&global_config, argv[1]);
        if (ns == nullptr) {
            out << "Cannot find subtree." << std::endl;
            return;
        }

        ns = ns->subspaces;

        if (ns == nullptr) {
            out << "Empty subtree." << std::endl;
            return;
        }
    }

    show_config_tree(out, ns, 0);
}

static void command_config_set(std::ostream& out, int argc, const char** argv)
{
    parameter_t* param;
    int value_i;
    float value_f;

    if (argc != 3) {
        out << "Usage: config_set /parameter/url value." << std::endl;
        return;
    }

    param = parameter_find(&global_config, argv[1]);

    if (param == NULL) {
        out << absl::StrFormat("Could not find parameter \"%s\"", argv[1]) << std::endl;
        return;
    }

    switch (param->type) {
        case _PARAM_TYPE_INTEGER:
            if (sscanf(argv[2], "%d", &value_i) == 1) {
                parameter_integer_set(param, value_i);
            } else {
                out << "Invalid value for integer parameter." << std::endl;
            }
            break;

        case _PARAM_TYPE_SCALAR:
            if (sscanf(argv[2], "%f", &value_f) == 1) {
                parameter_scalar_set(param, value_f);
            } else {
                out << "Invalid value for scalar parameter." << std::endl;
            }
            break;

        case _PARAM_TYPE_BOOLEAN:
            if (!strcmp(argv[2], "true")) {
                parameter_boolean_set(param, true);
            } else if (!strcmp(argv[2], "false")) {
                parameter_boolean_set(param, false);
            } else {
                out << "Invalid value for boolean parameter, must be 'true' or 'false'." << std::endl;
            }
            break;

        case _PARAM_TYPE_STRING:
            if (argc == 2) {
                parameter_string_set(param, argv[2]);
            } else {
                out << "Invalid value for string parameter, spaces are not supported." << std::endl;
            }
            break;

        default:
            out << absl::StrFormat("%s: unknown type %d", argv[1], param->type) << std::endl;
            break;
    }
}

absl::flat_hash_map<std::string, TerminalHandler> shell_commands{
    {"reboot", command_reboot},
    {"config_tree", command_config_tree},
    {"config_set", command_config_set},
};
