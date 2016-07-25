#include "cvra/cvra_motors.h"
#include "base/encoder.h"


void cvra_motor_left_wheel_set_velocity(void* motor, int32_t velocity)
{
    cvra_motor_t *dev = (cvra_motor_t *)motor;
    motor_manager_set_velocity(dev, "left-motor", velocity / dev->max_velocity);
}


void cvra_motor_right_wheel_set_velocity(void* motor, int32_t velocity)
{
    cvra_motor_t *dev = (cvra_motor_t *)motor;
    motor_manager_set_velocity(dev, "right-motor", velocity / dev->max_velocity);
}

int32_t cvra_encoder_get_left_ext(void *nothing)
{
    (void)nothing;
    return encoder_get_left();
}

int32_t cvra_encoder_get_right_ext(void *nothing)
{
    (void)nothing;
    return encoder_get_right();
}

