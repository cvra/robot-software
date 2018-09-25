#include "base.h"

static void wheel_set_speed(wheel_t* wheel, float speed);

void base_set_speed(base_t* base, float linear_x, float angular_z)
{
    float right_speed = 0.5f * (linear_x - angular_z);
    float left_speed = 0.5f * (linear_x + angular_z);

    wheel_set_speed(&base->right.back_wheel, right_speed);
    wheel_set_speed(&base->right.center_wheel, right_speed);
    wheel_set_speed(&base->right.front_wheel, right_speed);

    wheel_set_speed(&base->left.back_wheel, left_speed);
    wheel_set_speed(&base->left.center_wheel, left_speed);
    wheel_set_speed(&base->left.front_wheel, left_speed);
}

void wheel_set_speed(wheel_t* wheel, float speed)
{
    motor_driver_set_voltage(wheel->motor, wheel->speed_factor * speed);
}
