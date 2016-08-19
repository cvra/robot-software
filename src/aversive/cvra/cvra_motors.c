#include "cvra/cvra_motors.h"
#include "base/encoder.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

uint32_t left_encoder_prev, right_encoder_prev;
int32_t left_encoder_value, right_encoder_value;

void cvra_encoder_init(void)
{
    left_encoder_prev = encoder_get_left();
    right_encoder_prev = encoder_get_right();
    left_encoder_value = left_encoder_prev;
    right_encoder_value = right_encoder_prev;
}

void cvra_motor_left_wheel_set_velocity(void* motor, int32_t velocity)
{
    cvra_motor_t *dev = (cvra_motor_t *)motor;
    motor_manager_set_velocity(dev->m, "left-wheel", velocity / dev->max_velocity);
    chprintf((BaseSequentialStream *)&SD3, "Left vel %.2f\r\n", velocity / dev->max_velocity);
}

void cvra_motor_right_wheel_set_velocity(void* motor, int32_t velocity)
{
    cvra_motor_t *dev = (cvra_motor_t *)motor;
    motor_manager_set_velocity(dev->m, "right-wheel", velocity / dev->max_velocity);
    chprintf((BaseSequentialStream *)&SD3, "Right vel %.2f\r\n", velocity / dev->max_velocity);
}

int32_t cvra_encoder_get_left_ext(void *nothing)
{
    (void)nothing;
    uint32_t left_encoder = encoder_get_left();
    left_encoder_value += encoder_tick_diff(left_encoder_prev, left_encoder);
    left_encoder_prev = left_encoder;
    return left_encoder_value;
}

int32_t cvra_encoder_get_right_ext(void *nothing)
{
    (void)nothing;
    uint32_t right_encoder = encoder_get_right();
    right_encoder_value += encoder_tick_diff(right_encoder_prev, right_encoder);
    right_encoder_prev = right_encoder;
    return right_encoder_value;
}
