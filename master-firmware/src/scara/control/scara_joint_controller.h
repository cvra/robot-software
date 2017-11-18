#ifndef SCARA_CONTROL__JOINT_CONTROLLER_H
#define SCARA_CONTROL__JOINT_CONTROLLER_H

#include "scara/scara_hardware_interface.h"
#include "scara/scara_kinematics.h"

typedef struct {
    float length[2];
    shoulder_mode_t shoulder_mode;
} scara_joint_controller_t;

void scara_joint_controller_init(scara_joint_controller_t* controller);

void scara_joint_controller_set_geometry(scara_joint_controller_t *controller,
                                         float *length,
                                         shoulder_mode_t shoulder_mode);

scara_joint_setpoints_t
scara_joint_controller_process(scara_joint_controller_t *controller,
                               position_3d_t desired,
                               scara_joint_positions_t measured);

#endif
