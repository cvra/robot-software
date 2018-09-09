#include <ch.h>
#include <stdio.h>
#include <error/error.h>
#include "config.h"

parameter_namespace_t global_config;
parameter_namespace_t actuator_config;
parameter_namespace_t drive_config;

void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
    parameter_namespace_declare(&actuator_config, &global_config, "actuator");
    parameter_namespace_declare(&drive_config, &global_config, "drive");
}

static parameter_t* config_get_param(const char *id)
{
    parameter_t *p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        ERROR("Unknown parameter \"%s\"", id);
    }

    return p;
}

float config_get_scalar(const char *id)
{
    return parameter_scalar_get(config_get_param(id));
}

int config_get_integer(const char *id)
{
    return parameter_integer_get(config_get_param(id));
}

bool config_get_boolean(const char *id)
{
    return parameter_boolean_get(config_get_param(id));
}
