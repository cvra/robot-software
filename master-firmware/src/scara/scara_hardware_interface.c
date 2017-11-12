#include "scara_hardware_interface.h"

void scara_hw_shutdown_joints(scara_hardware_interface_t* hw_interface)
{
    hw_interface->z_joint.set_velocity(hw_interface->z_joint.args, 0);
    hw_interface->shoulder_joint.set_velocity(hw_interface->shoulder_joint.args, 0);
    hw_interface->elbow_joint.set_velocity(hw_interface->elbow_joint.args, 0);
}

scara_joint_positions_t scara_hw_read_joint_positions(scara_hardware_interface_t* hw_interface)
{
    scara_joint_positions_t joint_positions;

    joint_positions.z = hw_interface->z_joint.get_position(hw_interface->z_joint.args);
    joint_positions.shoulder = hw_interface->shoulder_joint.get_position(hw_interface->shoulder_joint.args);
    joint_positions.elbow = hw_interface->elbow_joint.get_position(hw_interface->elbow_joint.args);

    return joint_positions;
}
