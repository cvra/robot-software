#ifndef SCARA_CONTROL__INVERSE_KINEMATICS_CONTROLLER_H
#define SCARA_CONTROL__INVERSE_KINEMATICS_CONTROLLER_H

#include <pid/pid.h>
#include <scara/scara_waypoint.h>
#include <scara/scara_hardware_interface.h>

typedef struct {
    /* Control system */
    pid_ctrl_t x_pid;
    pid_ctrl_t y_pid;
} scara_ik_controller_t;

void scara_ik_controller_init(scara_ik_controller_t* controller);

scara_joint_setpoints_t
scara_ik_controller_process(scara_ik_controller_t *controller,
                            position_3d_t measured, position_3d_t desired,
                            scara_joint_positions_t joint_positions,
                            float *length);

#endif
