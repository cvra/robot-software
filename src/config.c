#include <ch.h>
#include <stdio.h>
#include "robot_parameters.h"
#include "config.h"

parameter_namespace_t global_config;

parameter_namespace_t actuator_config;

parameter_namespace_t master_config;

static parameter_t foo;

static parameter_namespace_t odometry_config;
static parameter_t odometry_track;
static parameter_t odometry_radius;



void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
    parameter_namespace_declare(&actuator_config, &global_config, "actuator");
    parameter_namespace_declare(&master_config, &global_config, "master");

    parameter_namespace_declare(&odometry_config, &master_config, "odometry");
    parameter_scalar_declare_with_default(&odometry_track,
                                          &odometry_config,
                                          "track",
                                          ROBOT_EXTERNAL_TRACK_LENGTH);

    parameter_scalar_declare_with_default(&odometry_radius,
                                          &odometry_config,
                                          "radius",
                                          ROBOT_EXTERNAL_WHEEL_RADIUS);

    parameter_scalar_declare(&foo, &master_config, "foo");
}

float config_get_scalar(const char *id)
{
    parameter_t *p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        char err_msg[64];
        snprintf(err_msg, sizeof err_msg, "Unknown param %s", id);
        chSysHalt(err_msg);
    }

    return parameter_scalar_get(p);
}
