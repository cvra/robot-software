#include <ch.h>
#include <stdio.h>
#include <string.h>

#include <error/error.h>
#include "config.h"

#include "config_private.h"

parameter_namespace_t global_config;
parameter_namespace_t actuator_config;
parameter_namespace_t master_config;

void config_init(void)
{
    config_master_init(); // Generated, see config_private.h

    // Initialize public facing namespaces from private config
    memcpy(&global_config, &config.ns, sizeof(parameter_namespace_t));
    memcpy(&master_config, &config.master.ns, sizeof(parameter_namespace_t));
    parameter_namespace_declare(&actuator_config, &global_config, "actuator");
}

static parameter_t* config_get_param(const char* id)
{
    parameter_t* p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        ERROR("Unknown parameter \"%s\"", id);
    }

    return p;
}

float config_get_scalar(const char* id)
{
    return parameter_scalar_get(config_get_param(id));
}

int config_get_integer(const char* id)
{
    return parameter_integer_get(config_get_param(id));
}

bool config_get_boolean(const char* id)
{
    return parameter_boolean_get(config_get_param(id));
}
