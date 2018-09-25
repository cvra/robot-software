#include "base.h"

#include <math.h>

static void base_advance(base_t* base, float speed);
static void base_rotate(base_t* base, float speed);
static void wheel_set_speed(wheel_t* wheel, float speed);
static float clamp(float value, float max);

void base_set_speed(base_t* base, float linear_x, float angular_z)
{
    if (fabsf(linear_x) >= fabsf(angular_z)) {
        base_advance(base, linear_x);
    } else {
        base_rotate(base, angular_z);
    }
}

void base_advance(base_t* base, float speed)
{
    float right_speed = speed;
    float left_speed = speed;

    wheel_set_speed(&base->right.back_wheel, right_speed);
    wheel_set_speed(&base->right.center_wheel, right_speed);
    wheel_set_speed(&base->right.front_wheel, right_speed);

    wheel_set_speed(&base->left.back_wheel, left_speed);
    wheel_set_speed(&base->left.center_wheel, left_speed);
    wheel_set_speed(&base->left.front_wheel, left_speed);
}

void base_rotate(base_t* base, float speed)
{
    float right_speed = - speed;
    float left_speed = speed;

    wheel_set_speed(&base->right.back_wheel, right_speed);
    wheel_set_speed(&base->right.center_wheel, right_speed);
    wheel_set_speed(&base->right.front_wheel, right_speed);

    wheel_set_speed(&base->left.back_wheel, left_speed);
    wheel_set_speed(&base->left.center_wheel, left_speed);
    wheel_set_speed(&base->left.front_wheel, left_speed);
}

void wheel_set_speed(wheel_t* wheel, float speed)
{
    motor_driver_set_voltage(wheel->motor, clamp(wheel->speed_factor * speed, WHEEL_MAX_SPEED));
}

float clamp(float value, float max)
{
    if (value > max) return max;
    if (value < -max) return -max;
    return value;
}
