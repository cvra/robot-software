#ifndef SCARA_CONTROL_INVERSE_KINEMATICS_CONTROLLER_H
#define SCARA_CONTROL_INVERSE_KINEMATICS_CONTROLLER_H

#include <pid/pid.h>
#include <scara/scara_waypoint.h>
#include <scara/scara_hardware_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_ctrl_t x_pid;
    pid_ctrl_t y_pid;

    float length[2];
} scara_ik_controller_t;

void scara_ik_controller_init(scara_ik_controller_t* controller);

void scara_ik_controller_set_geometry(scara_ik_controller_t* controller,
                                      float* length);

scara_joint_setpoints_t
scara_ik_controller_process(scara_ik_controller_t* controller,
                            position_3d_t desired,
                            scara_joint_positions_t measured);

#ifdef __cplusplus
}
#endif

#endif /* SCARA_CONTROL_INVERSE_KINEMATICS_CONTROLLER_H */
