#include <ch.h>
#include <stdio.h>
#include "robot_parameters.h"
#include "config.h"

parameter_namespace_t global_config;

parameter_namespace_t actuator_config;

parameter_namespace_t master_config;

static parameter_namespace_t odometry_config;
static parameter_t odometry_track;
static parameter_t odometry_radius;

static parameter_namespace_t aversive_config;
static parameter_namespace_t aversive_control;
static parameter_namespace_t aversive_angle;
static parameter_t aversive_angle_kp;
static parameter_t aversive_angle_ki;
static parameter_t aversive_angle_kd;
static parameter_t aversive_angle_ilimit;
static parameter_namespace_t aversive_distance;
static parameter_t aversive_distance_kp;
static parameter_t aversive_distance_ki;
static parameter_t aversive_distance_kd;
static parameter_t aversive_distance_ilimit;

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

    parameter_namespace_declare(&aversive_config, &master_config, "aversive");
    parameter_namespace_declare(&aversive_control, &aversive_config, "control");
    parameter_namespace_declare(&aversive_angle, &aversive_control, "angle");
    parameter_scalar_declare(&aversive_angle_kp, &aversive_angle, "kp");
    parameter_scalar_declare(&aversive_angle_ki, &aversive_angle, "ki");
    parameter_scalar_declare(&aversive_angle_kd, &aversive_angle, "kd");
    parameter_scalar_declare(&aversive_angle_ilimit, &aversive_angle, "ilimit");
    parameter_namespace_declare(&aversive_distance, &aversive_control, "distance");
    parameter_scalar_declare(&aversive_distance_kp, &aversive_distance, "kp");
    parameter_scalar_declare(&aversive_distance_ki, &aversive_distance, "ki");
    parameter_scalar_declare(&aversive_distance_kd, &aversive_distance, "kd");
    parameter_scalar_declare(&aversive_distance_ilimit, &aversive_distance, "ilimit");
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
