#ifndef SCARA_HARDWARE_INTERFACE_H
#define SCARA_HARDWARE_INTERFACE_H

#include "joint.h"

typedef struct {
    joint_t z_joint;
    joint_t shoulder_joint;
    joint_t elbow_joint;
} scara_hardware_interface_t;

#endif
