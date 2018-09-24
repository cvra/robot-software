#include "base.h"

void base_set_speed(base_t* base, float linear_x, float angular_z)
{
    float right_voltage = 0.5f * (linear_x - angular_z);
    float left_voltage = 0.5f * (linear_x + angular_z);

    motor_driver_set_voltage(base->right.back_wheel.motor, right_voltage);
    motor_driver_set_voltage(base->right.center_wheel.motor, right_voltage);
    motor_driver_set_voltage(base->right.front_wheel.motor, right_voltage);

    motor_driver_set_voltage(base->left.back_wheel.motor, left_voltage);
    motor_driver_set_voltage(base->left.center_wheel.motor, left_voltage);
    motor_driver_set_voltage(base->left.front_wheel.motor, left_voltage);
}
