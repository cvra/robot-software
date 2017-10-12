#include <ch.h>
#include <stdio.h>
#include <error/error.h>
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

    struct {
        parameter_namespace_t ns;
        struct {
            parameter_namespace_t ns;
            struct {
                parameter_namespace_t ns;
                parameter_t init;
                parameter_t slow;
                parameter_t fast;
            } speed, acceleration;
        } angle, distance;
    } trajectories;
} aversive;

static parameter_namespace_t beacon_config;
static parameter_t beacon_reflector_radius, beacon_angular_offset;

#ifdef DEBRA
static parameter_namespace_t arms_config, arms_right_config, arms_left_config, motor_offsets_config;
static parameter_t upperarm_length, forearm_length, wrist_to_hand_length;
static parameter_t left_offset_x, left_offset_y, left_offset_a;
static parameter_t right_offset_x, right_offset_y, right_offset_a;
static parameter_t left_z_offset, left_shoulder_offset, left_elbow_offset;
static parameter_t right_z_offset, right_shoulder_offset, right_elbow_offset;

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
        } x, y, heading, pitch;
    } control;
} left_arm, right_arm;
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

    parameter_namespace_declare(&aversive.trajectories.ns, &aversive.ns, "trajectories");
    parameter_namespace_declare(&aversive.trajectories.angle.ns,
                                &aversive.trajectories.ns, "angle");
    parameter_namespace_declare(&aversive.trajectories.angle.speed.ns,
                                &aversive.trajectories.angle.ns, "speed");
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.speed.init,
                                          &aversive.trajectories.angle.speed.ns,
                                          "init", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.speed.slow,
                                          &aversive.trajectories.angle.speed.ns,
                                          "slow", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.speed.fast,
                                          &aversive.trajectories.angle.speed.ns,
                                          "fast", 0.);
    parameter_namespace_declare(&aversive.trajectories.angle.acceleration.ns,
                                &aversive.trajectories.angle.ns, "acceleration");
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.acceleration.init,
                                          &aversive.trajectories.angle.acceleration.ns,
                                          "init", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.acceleration.slow,
                                          &aversive.trajectories.angle.acceleration.ns,
                                          "slow", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.angle.acceleration.fast,
                                          &aversive.trajectories.angle.acceleration.ns,
                                          "fast", 0.);
    parameter_namespace_declare(&aversive.trajectories.distance.ns,
                                &aversive.trajectories.ns, "distance");
    parameter_namespace_declare(&aversive.trajectories.distance.speed.ns,
                                &aversive.trajectories.distance.ns, "speed");
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.speed.init,
                                          &aversive.trajectories.distance.speed.ns,
                                          "init", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.speed.slow,
                                          &aversive.trajectories.distance.speed.ns,
                                          "slow", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.speed.fast,
                                          &aversive.trajectories.distance.speed.ns,
                                          "fast", 0.);
    parameter_namespace_declare(&aversive.trajectories.distance.acceleration.ns,
                                &aversive.trajectories.distance.ns, "acceleration");
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.acceleration.init,
                                          &aversive.trajectories.distance.acceleration.ns,
                                          "init", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.acceleration.slow,
                                          &aversive.trajectories.distance.acceleration.ns,
                                          "slow", 0.);
    parameter_scalar_declare_with_default(&aversive.trajectories.distance.acceleration.fast,
                                          &aversive.trajectories.distance.acceleration.ns,
                                          "fast", 0.);
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
    parameter_scalar_declare(&right_z_offset, &motor_offsets_config, "right-z");
    parameter_scalar_declare(&right_shoulder_offset, &motor_offsets_config, "right-shoulder");
    parameter_scalar_declare(&right_elbow_offset, &motor_offsets_config, "right-elbow");

    parameter_namespace_declare(&left_arm.ns, &master_config, "left_arm");
    parameter_namespace_declare(&left_arm.control.ns, &left_arm.ns, "control");
    parameter_namespace_declare(&left_arm.control.x.ns, &left_arm.control.ns, "x");
    parameter_scalar_declare(&left_arm.control.x.kp, &left_arm.control.x.ns, "kp");
    parameter_scalar_declare(&left_arm.control.x.ki, &left_arm.control.x.ns, "ki");
    parameter_scalar_declare(&left_arm.control.x.kd, &left_arm.control.x.ns, "kd");
    parameter_scalar_declare(&left_arm.control.x.ilimit, &left_arm.control.x.ns, "ilimit");
    parameter_namespace_declare(&left_arm.control.y.ns, &left_arm.control.ns, "y");
    parameter_scalar_declare(&left_arm.control.y.kp, &left_arm.control.y.ns, "kp");
    parameter_scalar_declare(&left_arm.control.y.ki, &left_arm.control.y.ns, "ki");
    parameter_scalar_declare(&left_arm.control.y.kd, &left_arm.control.y.ns, "kd");
    parameter_scalar_declare(&left_arm.control.y.ilimit, &left_arm.control.y.ns, "ilimit");
    parameter_namespace_declare(&left_arm.control.heading.ns, &left_arm.control.ns, "heading");
    parameter_scalar_declare(&left_arm.control.heading.kp, &left_arm.control.heading.ns, "kp");
    parameter_scalar_declare(&left_arm.control.heading.ki, &left_arm.control.heading.ns, "ki");
    parameter_scalar_declare(&left_arm.control.heading.kd, &left_arm.control.heading.ns, "kd");
    parameter_scalar_declare(&left_arm.control.heading.ilimit, &left_arm.control.heading.ns, "ilimit");
    parameter_namespace_declare(&left_arm.control.pitch.ns, &left_arm.control.ns, "pitch");
    parameter_scalar_declare(&left_arm.control.pitch.kp, &left_arm.control.pitch.ns, "kp");
    parameter_scalar_declare(&left_arm.control.pitch.ki, &left_arm.control.pitch.ns, "ki");
    parameter_scalar_declare(&left_arm.control.pitch.kd, &left_arm.control.pitch.ns, "kd");
    parameter_scalar_declare(&left_arm.control.pitch.ilimit, &left_arm.control.pitch.ns, "ilimit");

    parameter_namespace_declare(&right_arm.ns, &master_config, "right_arm");
    parameter_namespace_declare(&right_arm.control.ns, &right_arm.ns, "control");
    parameter_namespace_declare(&right_arm.control.x.ns, &right_arm.control.ns, "x");
    parameter_scalar_declare(&right_arm.control.x.kp, &right_arm.control.x.ns, "kp");
    parameter_scalar_declare(&right_arm.control.x.ki, &right_arm.control.x.ns, "ki");
    parameter_scalar_declare(&right_arm.control.x.kd, &right_arm.control.x.ns, "kd");
    parameter_scalar_declare(&right_arm.control.x.ilimit, &right_arm.control.x.ns, "ilimit");
    parameter_namespace_declare(&right_arm.control.y.ns, &right_arm.control.ns, "y");
    parameter_scalar_declare(&right_arm.control.y.kp, &right_arm.control.y.ns, "kp");
    parameter_scalar_declare(&right_arm.control.y.ki, &right_arm.control.y.ns, "ki");
    parameter_scalar_declare(&right_arm.control.y.kd, &right_arm.control.y.ns, "kd");
    parameter_scalar_declare(&right_arm.control.y.ilimit, &right_arm.control.y.ns, "ilimit");
    parameter_namespace_declare(&right_arm.control.heading.ns, &right_arm.control.ns, "heading");
    parameter_scalar_declare(&right_arm.control.heading.kp, &right_arm.control.heading.ns, "kp");
    parameter_scalar_declare(&right_arm.control.heading.ki, &right_arm.control.heading.ns, "ki");
    parameter_scalar_declare(&right_arm.control.heading.kd, &right_arm.control.heading.ns, "kd");
    parameter_scalar_declare(&right_arm.control.heading.ilimit, &right_arm.control.heading.ns, "ilimit");
    parameter_namespace_declare(&right_arm.control.pitch.ns, &right_arm.control.ns, "pitch");
    parameter_scalar_declare(&right_arm.control.pitch.kp, &right_arm.control.pitch.ns, "kp");
    parameter_scalar_declare(&right_arm.control.pitch.ki, &right_arm.control.pitch.ns, "ki");
    parameter_scalar_declare(&right_arm.control.pitch.kd, &right_arm.control.pitch.ns, "kd");
    parameter_scalar_declare(&right_arm.control.pitch.ilimit, &right_arm.control.pitch.ns, "ilimit");
#endif
}

float config_get_scalar(const char *id)
{
    parameter_t *p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        ERROR("Unknown parameter \"%s\"", id);
    }

    return parameter_scalar_get(p);
}

int config_get_integer(const char *id)
{
    parameter_t *p;

    p = parameter_find(&global_config, id);

    if (p == NULL) {
        ERROR("Unknown parameter \"%s\"", id);
    }

    return parameter_integer_get(p);
}
