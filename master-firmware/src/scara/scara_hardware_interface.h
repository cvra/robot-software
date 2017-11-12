#ifndef SCARA_HARDWARE_INTERFACE_H
#define SCARA_HARDWARE_INTERFACE_H

#include "joint.h"

typedef struct {
    joint_t z_joint;
    joint_t shoulder_joint;
    joint_t elbow_joint;
} scara_hardware_interface_t;

typedef struct {
    float z;
    float shoulder;
    float elbow;
} scara_joint_positions_t;

void scara_hw_shutdown_joints(scara_hardware_interface_t* hw_interface);

scara_joint_positions_t scara_hw_read_joint_positions(scara_hardware_interface_t* hw_interface);

#endif
