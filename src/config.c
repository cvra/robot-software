#include <ch.h>
#include <stdio.h>
#include "robot_parameters.h"
#include "config.h"
#include "tracy-the-trajectory-tracker/src/trajectory_tracking.h"

parameter_namespace_t global_config;

parameter_namespace_t actuator_config;

static parameter_namespace_t master_config;

static parameter_t foo;

static parameter_namespace_t odometry_config;
static parameter_t odometry_wheel_base;
static parameter_t odometry_left_radius;
static parameter_t odometry_right_radius;
static parameter_t odometry_left_wheel_direction;
static parameter_t odometry_right_wheel_direction;

static parameter_namespace_t differential_base_config;
static parameter_t differential_base_wheel_base;
static parameter_t differential_base_left_radius;
static parameter_t differential_base_right_radius;

static parameter_namespace_t tracy_config;
static parameter_t tracy_g;
static parameter_t tracy_damping_coef;


void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
    parameter_namespace_declare(&actuator_config, &global_config, "actuator");
    parameter_namespace_declare(&master_config, &global_config, "master");

    parameter_namespace_declare(&odometry_config, &master_config, "odometry");
    parameter_scalar_declare_with_default(&odometry_wheel_base,
                                          &odometry_config,
                                          "wheelbase",
                                          ROBOT_EXTERNAL_WHEELBASE);

    parameter_scalar_declare_with_default(&odometry_right_radius,
                                          &odometry_config,
                                          "radius_right",
                                          ROBOT_RIGHT_EXTERNAL_WHEEL_RADIUS);

    parameter_scalar_declare_with_default(&odometry_left_radius,
                                          &odometry_config,
                                          "radius_left",
                                          ROBOT_LEFT_EXTERNAL_WHEEL_RADIUS);

    parameter_integer_declare_with_default(&odometry_right_wheel_direction,
                                           &odometry_config,
                                           "right_wheel_direction",
                                           ROBOT_RIGHT_WHEEL_DIRECTION);

    parameter_integer_declare_with_default(&odometry_left_wheel_direction,
                                           &odometry_config,
                                           "left_wheel_direction",
                                           ROBOT_LEFT_WHEEL_DIRECTION);

    parameter_namespace_declare(&differential_base_config, &master_config, "differential_base");
    parameter_scalar_declare_with_default(&differential_base_wheel_base,
                                          &differential_base_config,
                                          "wheelbase",
                                          ROBOT_EXTERNAL_WHEELBASE);

    parameter_scalar_declare_with_default(&differential_base_right_radius,
                                          &differential_base_config,
                                          "radius_right",
                                          ROBOT_RIGHT_MOTOR_WHEEL_RADIUS);

    parameter_scalar_declare_with_default(&differential_base_left_radius,
                                          &differential_base_config,
                                          "radius_left",
                                          ROBOT_LEFT_MOTOR_WHEEL_RADIUS);

    parameter_namespace_declare(&tracy_config, &master_config, "tracy");
    parameter_scalar_declare_with_default(&tracy_g, &tracy_config, "g",
                                          DEFAULT_PARAM_G);

    parameter_scalar_declare_with_default(&tracy_damping_coef, &tracy_config, "damping",
                                          DEFAULT_PARAM_DAMPING_COEFF);

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
