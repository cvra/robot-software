#include "config.h"

parameter_namespace_t global_config;

void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
}
