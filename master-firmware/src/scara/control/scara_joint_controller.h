#ifndef SCARA_CONTROL__JOINT_CONTROLLER_H
#define SCARA_CONTROL__JOINT_CONTROLLER_H

#include "scara/scara_hardware_interface.h"

scara_joint_setpoints_t
scara_joint_controller_process(scara_joint_positions_t desired,
                               scara_joint_positions_t measured);

#endif
