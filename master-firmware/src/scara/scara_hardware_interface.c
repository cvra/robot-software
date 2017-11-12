#include "scara_hardware_interface.h"

void scara_hw_shutdown_joints(scara_hardware_interface_t* hw_interface)
{
    hw_interface->z_joint.set_velocity(hw_interface->z_joint.args, 0);
    hw_interface->shoulder_joint.set_velocity(hw_interface->shoulder_joint.args, 0);
    hw_interface->elbow_joint.set_velocity(hw_interface->elbow_joint.args, 0);
}
