#include "base.h"
#include "pca9685_pwm.h"

#include <math.h>

static void base_advance(base_t* base, float speed);
static void base_rotate(base_t* base, float speed);
static void wheel_set_speed(wheel_t* wheel, float speed);
static void wheel_set_angle(wheel_t* wheel, float angle);
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

    wheel_set_angle(&base->right.back_wheel, 0);
    wheel_set_angle(&base->right.center_wheel, 0);
    wheel_set_angle(&base->right.front_wheel, 0);
    wheel_set_speed(&base->right.back_wheel, right_speed);
    wheel_set_speed(&base->right.center_wheel, right_speed);
    wheel_set_speed(&base->right.front_wheel, right_speed);

    wheel_set_angle(&base->left.back_wheel, 0);
    wheel_set_angle(&base->left.center_wheel, 0);
    wheel_set_angle(&base->left.front_wheel, 0);
    wheel_set_speed(&base->left.back_wheel, left_speed);
    wheel_set_speed(&base->left.center_wheel, left_speed);
    wheel_set_speed(&base->left.front_wheel, left_speed);
}

void base_rotate(base_t* base, float speed)
{
    float right_speed = - speed;
    float left_speed = speed;

    wheel_set_angle(&base->right.back_wheel, -45.f);
    wheel_set_angle(&base->right.center_wheel, 0);
    wheel_set_angle(&base->right.front_wheel, 45.f);
    wheel_set_speed(&base->right.back_wheel, right_speed);
    wheel_set_speed(&base->right.center_wheel, right_speed);
    wheel_set_speed(&base->right.front_wheel, right_speed);

    wheel_set_angle(&base->left.back_wheel, 45.f);
    wheel_set_angle(&base->left.center_wheel, 0);
    wheel_set_angle(&base->left.front_wheel, -45.f);
    wheel_set_speed(&base->left.back_wheel, left_speed);
    wheel_set_speed(&base->left.center_wheel, left_speed);
    wheel_set_speed(&base->left.front_wheel, left_speed);
}

void wheel_set_speed(wheel_t* wheel, float speed)
{
    motor_driver_set_voltage(wheel->motor, clamp(wheel->speed_factor * speed, WHEEL_MAX_SPEED));
}

void wheel_set_angle(wheel_t* wheel, float angle)
{
    float pulse_width = angle * WHEEL_STEERING_90_DEG / 90.f;
    pca9685_pwm_set_pulse_width(wheel->servo, wheel->steering_center + pulse_width);
}

float clamp(float value, float max)
{
    if (value > max) return max;
    if (value < -max) return -max;
    return value;
}
