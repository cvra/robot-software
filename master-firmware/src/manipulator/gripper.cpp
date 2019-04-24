#include "manipulator/gripper.h"

#include "main.h"
#include "motor_manager.h"

namespace manipulator {
void Gripper::configure(float new_release_voltage, float new_acquire_voltage)
{
    release_voltage = new_release_voltage;
    acquire_voltage = new_acquire_voltage;
}
void Gripper::release() const
{
    motor_manager_set_voltage(&motor_manager, pumps[0], release_voltage);
    motor_manager_set_voltage(&motor_manager, pumps[1], release_voltage);
}
void Gripper::acquire() const
{
    motor_manager_set_voltage(&motor_manager, pumps[0], acquire_voltage);
    motor_manager_set_voltage(&motor_manager, pumps[1], acquire_voltage);
}
void Gripper::disable() const
{
    motor_manager_set_voltage(&motor_manager, pumps[0], 0);
    motor_manager_set_voltage(&motor_manager, pumps[1], 0);
}
} // namespace manipulator
