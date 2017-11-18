#include "scara_inverse_kinematics_controller.h"

#include <scara/scara_jacobian.h>

scara_joint_setpoints_t
scara_ik_controller_process(position_3d_t measured, position_3d_t desired,
                            scara_joint_positions_t joint_positions,
                            float *length, pid_ctrl_t x_pid, pid_ctrl_t y_pid)
{
    float consign_x = pid_process(&x_pid, measured.x - desired.x);
    float consign_y = pid_process(&y_pid, measured.y - desired.y);

    float velocity_alpha, velocity_beta;
    scara_jacobian_compute(consign_x, consign_y, joint_positions.shoulder,
                           joint_positions.elbow, length[0], length[1],
                           &velocity_alpha, &velocity_beta);

    scara_joint_setpoints_t joint_setpoints = {
        .z = {POSITION, desired.z},
        .shoulder = {VELOCITY, velocity_alpha},
        .elbow = {VELOCITY, velocity_beta}};

    return joint_setpoints;
}
