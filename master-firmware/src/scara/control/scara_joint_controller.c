#include "scara_joint_controller.h"

scara_joint_setpoints_t scara_joint_controller_process(scara_joint_positions_t desired,
                                                       scara_joint_positions_t measured)
{
    (void)measured;

    scara_joint_setpoints_t setpoints = {
        .z = {POSITION, desired.z},
        .shoulder = {POSITION, desired.shoulder},
        .elbow = {POSITION, desired.elbow}
    };

    return setpoints;
}
