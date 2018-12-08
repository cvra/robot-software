#include "scara_joint_controller.h"

#include <error/error.h>

#include <string.h>

void scara_joint_controller_init(scara_joint_controller_t* controller)
{
    memset(controller, 0, sizeof(scara_joint_controller_t));
}

void scara_joint_controller_set_geometry(scara_joint_controller_t* controller,
                                         float* length,
                                         shoulder_mode_t shoulder_mode)
{
    controller->length[0] = length[0];
    controller->length[1] = length[1];
    controller->shoulder_mode = shoulder_mode;
}

scara_joint_setpoints_t
scara_joint_controller_process(scara_joint_controller_t* controller,
                               position_3d_t desired,
                               scara_joint_positions_t measured)
{
    (void)measured;

    float alpha, beta;
    bool solution_found = scara_compute_joint_angles(
        desired, controller->shoulder_mode, controller->length, &alpha, &beta);

    if (solution_found) {
        DEBUG("Inverse kinematics: Found a solution");
        scara_joint_setpoints_t setpoint = {
            .z = {POSITION, desired.z},
            .shoulder = {POSITION, alpha},
            .elbow = {POSITION, beta}};
        return setpoint;
    } else {
        DEBUG("Inverse kinematics: Found no solution, disabling the arm");
        scara_joint_setpoints_t null_speed_setpoint = {
            .z = {VELOCITY, 0.0},
            .shoulder = {VELOCITY, 0.0},
            .elbow = {VELOCITY, 0.0}};
        return null_speed_setpoint;
    }
}
