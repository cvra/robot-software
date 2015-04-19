#include "config.h"

parameter_namespace_t global_config;

static parameter_namespace_t master_config;
static parameter_t foo;

void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
    parameter_namespace_declare(&master_config, &global_config, "master");
    parameter_scalar_declare(&foo, &master_config, "foo");
}
