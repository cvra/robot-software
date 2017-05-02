#include <ch.h>
#include <stdio.h>
#include "config.h"

parameter_namespace_t global_config;

parameter_namespace_t actuator_config;

parameter_namespace_t master_config;

static parameter_namespace_t odometry_config;
static parameter_t robot_size, robot_alignement_length, opponent_size, calib_dir;
static parameter_t odometry_ticks, odometry_track, odometry_left_corr, odometry_right_corr;

static struct {
    parameter_namespace_t ns;
    struct {
        parameter_namespace_t ns;
        struct {
            parameter_namespace_t ns;
            parameter_t kp;
            parameter_t ki;
            parameter_t kd;
            parameter_t ilimit;
        } angle, distance;
    } control;
} aversive;

static parameter_namespace_t beacon_config;
static parameter_t beacon_reflector_radius, beacon_angular_offset;

#ifdef DEBRA
static parameter_namespace_t arms_config, arms_right_config, arms_left_config, motor_offsets_config;
static parameter_t upperarm_length, forearm_length, wrist_to_hand_length;
static parameter_t left_offset_x, left_offset_y, left_offset_a;
static parameter_t right_offset_x, right_offset_y, right_offset_a;
static parameter_t left_z_offset, left_shoulder_offset, left_elbow_offset, left_wrist_offset;
static parameter_t right_z_offset, right_shoulder_offset, right_elbow_offset, right_wrist_offset;
#endif

void config_init(void)
{
    parameter_namespace_declare(&global_config, NULL, NULL);
    parameter_namespace_declare(&actuator_config, &global_config, "actuator");
    parameter_namespace_declare(&master_config, &global_config, "master");

    parameter_integer_declare(&robot_size, &master_config, "robot_size_x_mm");
    parameter_integer_declare(&robot_alignement_length, &master_config, "robot_alignment_length_mm");
    parameter_integer_declare(&opponent_size, &master_config, "opponent_size_x_mm_default");
    parameter_integer_declare(&calib_dir, &master_config, "calibration_direction");

    parameter_namespace_declare(&odometry_config, &master_config, "odometry");
    parameter_scalar_declare(&odometry_ticks, &odometry_config, "external_encoder_ticks_per_mm");
    parameter_scalar_declare(&odometry_track, &odometry_config, "external_track_mm");
    parameter_scalar_declare(&odometry_left_corr, &odometry_config, "left_wheel_correction_factor");
    parameter_scalar_declare(&odometry_right_corr, &odometry_config,
                             "right_wheel_correction_factor");

    parameter_namespace_declare(&beacon_config, &master_config, "beacon");
    parameter_scalar_declare(&beacon_reflector_radius, &beacon_config, "reflector_radius");
    parameter_scalar_declare(&beacon_angular_offset, &beacon_config, "angular_offset");

    parameter_namespace_declare(&aversive.ns, &master_config, "aversive");

    parameter_namespace_declare(&aversive.control.ns, &aversive.ns, "control");
    parameter_namespace_declare(&aversive.control.angle.ns, &aversive.control.ns, "angle");
    parameter_scalar_declare(&aversive.control.angle.kp, &aversive.control.angle.ns, "kp");
    parameter_scalar_declare(&aversive.control.angle.ki, &aversive.control.angle.ns, "ki");
    parameter_scalar_declare(&aversive.control.angle.kd, &aversive.control.angle.ns, "kd");
    parameter_scalar_declare(&aversive.control.angle.ilimit, &aversive.control.angle.ns, "ilimit");
    parameter_namespace_declare(&aversive.control.distance.ns, &aversive.control.ns, "distance");
    parameter_scalar_declare(&aversive.control.distance.kp, &aversive.control.distance.ns, "kp");
    parameter_scalar_declare(&aversive.control.distance.ki, &aversive.control.distance.ns, "ki");
    parameter_scalar_declare(&aversive.control.distance.kd, &aversive.control.distance.ns, "kd");
    parameter_scalar_declare(&aversive.control.distance.ilimit,
                             &aversive.control.distance.ns,
                             "ilimit");

#ifdef DEBRA
    parameter_namespace_declare(&arms_config, &master_config, "arms");

    parameter_scalar_declare(&upperarm_length, &arms_config, "upperarm_length");
    parameter_scalar_declare(&forearm_length, &arms_config, "forearm_length");
    parameter_scalar_declare(&wrist_to_hand_length, &arms_config, "wrist_to_hand_length");

    parameter_namespace_declare(&arms_left_config, &arms_config, "left");
    parameter_scalar_declare(&left_offset_x, &arms_left_config, "offset_x");
    parameter_scalar_declare(&left_offset_y, &arms_left_config, "offset_y");
    parameter_scalar_declare(&left_offset_a, &arms_left_config, "offset_a");

    parameter_namespace_declare(&arms_right_config, &arms_config, "right");
    parameter_scalar_declare(&right_offset_x, &arms_right_config, "offset_x");
    parameter_scalar_declare(&right_offset_y, &arms_right_config, "offset_y");
    parameter_scalar_declare(&right_offset_a, &arms_right_config, "offset_a");

    parameter_namespace_declare(&motor_offsets_config, &arms_config, "motor_offsets");
    parameter_scalar_declare(&left_z_offset, &motor_offsets_config, "left-z");
    parameter_scalar_declare(&left_shoulder_offset, &motor_offsets_config, "left-shoulder");
    parameter_scalar_declare(&left_elbow_offset, &motor_offsets_config, "left-elbow");
    parameter_scalar_declare(&left_wrist_offset, &motor_offsets_config, "left-wrist");
    parameter_scalar_declare(&right_z_offset, &motor_offsets_config, "right-z");
    parameter_scalar_declare(&right_shoulder_offset, &motor_offsets_config, "right-shoulder");
    parameter_scalar_declare(&right_elbow_offset, &motor_offsets_config, "right-elbow");
    parameter_scalar_declare(&right_wrist_offset, &motor_offsets_config, "right-wrist");
#endif
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

int config_get_integer(const char *id)
{
    parameter_t *p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        char err_msg[64];
        snprintf(err_msg, sizeof err_msg, "Unknown param %s", id);
        chSysHalt(err_msg);
    }

    return parameter_integer_get(p);
}
