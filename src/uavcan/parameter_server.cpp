#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/param_server.hpp>
#include "main.h"
#include "parameter_server.hpp"

static const char *exposed_parameters[] = {
    "/publish/imu",
    "/publish/attitude",
    "/publish/range",
    "/ahrs/beta",
};

static const size_t exposed_parameters_len = sizeof(exposed_parameters) /
                                             sizeof(exposed_parameters[0]);

class ParamManager : public uavcan::IParamManager
{
public:
/**
 * Copy the parameter name to @ref out_name if it exists, otherwise do nothing.
 */
    void getParamNameByIndex(Index index, Name& out_name) const
    {
        if (index < exposed_parameters_len) {
            out_name = exposed_parameters[index];
        }
    }
/**
 * Assign by name if exists.
 */
    void assignParamValue(const Name& name, const Value& value)
    {
        parameter_t *p = parameter_find(&parameter_root, name.c_str());

        /* If the parameter was not found, abort. */
        if (p == nullptr) {
            return;
        }

        if (p->type == _PARAM_TYPE_SCALAR) {
            auto *val = value.as<Value::Tag::real_value>();
            if (val) {
                parameter_scalar_set(p, *val);
            }
        } else if (p->type == _PARAM_TYPE_INTEGER) {
            auto *val = value.as<Value::Tag::integer_value>();
            if (val && *val > INT32_MIN && *val < INT32_MAX) {
                parameter_integer_set(p, *val);
            }
        } else if (p->type == _PARAM_TYPE_BOOLEAN) {
            auto *val = value.as<Value::Tag::boolean_value>();
            if (val) {
                parameter_boolean_set(p, *val);
            }
        } else if (p->type == _PARAM_TYPE_STRING) {
            auto *val = value.as<Value::Tag::string_value>();
            if (val) {
                parameter_string_set(p, val->c_str());
            }
        }
    }

/**
 * Read by name if exists, otherwise do nothing.
 */
    void readParamValue(const Name& name, Value& out_value) const
    {
        parameter_t *p = parameter_find(&parameter_root, name.c_str());

        /* If the parameter was not found, abort. */
        if (p == nullptr) {
            return;
        }

        if (!parameter_defined(p)) {
            return;
        }

        if (p->type == _PARAM_TYPE_SCALAR) {
            out_value.to<Value::Tag::real_value>() = parameter_scalar_read(p);
        } else if (p->type == _PARAM_TYPE_INTEGER) {
            out_value.to<Value::Tag::integer_value>() = parameter_integer_read(p);
        } else if (p->type == _PARAM_TYPE_BOOLEAN) {
            out_value.to<Value::Tag::boolean_value>() = parameter_boolean_read(p);
        } else if (p->type == _PARAM_TYPE_STRING) {
            char name[93];
            parameter_string_read(p, name, sizeof(p));
            out_value.to<Value::Tag::string_value>() = name;
        }
    }

/**
 * Save all params to non-volatile storage.
 * @return Negative if failed.
 *
 * @note Not supported on motor board, always return -1.
 */
    int saveAllParams()
    {
        /* Not supported. */
        return -uavcan::ErrDriver;
    }

/**
 * Clear the non-volatile storage.
 * @return Negative if failed.
 *
 * @note Not supported on motor board, always return -1.
 */
    int eraseAllParams()
    {
        /* Not supported. */
        return -uavcan::ErrDriver;
    }
};

int parameter_server_start(Node &node)
{
    static uavcan::ParamServer server(node);
    static ParamManager manager;
    return server.start(&manager);
}
