#include "cvra_motors.h"
#include "base/encoder.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define MAX_MOTOR_TORQUE_SCALE 1000.f

uint32_t left_encoder_prev, right_encoder_prev;
int32_t left_encoder_value, right_encoder_value;

void cvra_encoder_init(void)
{
    left_encoder_prev = encoder_get_left();
    right_encoder_prev = encoder_get_right();
    left_encoder_value = left_encoder_prev;
    right_encoder_value = right_encoder_prev;
}

static void cvra_motor_set_torque(const char *id, void *motor, int32_t torque)
{
    cvra_motor_t *dev = (cvra_motor_t *)motor;

    float vel = torque * dev->direction / MAX_MOTOR_TORQUE_SCALE;

    motor_manager_set_torque(dev->m, id, vel);
}

void cvra_motor_left_wheel_set_torque(void* motor, int32_t torque)
{
    cvra_motor_set_torque("left-wheel", motor, torque);
}

void cvra_motor_right_wheel_set_torque(void* motor, int32_t torque)
{
    cvra_motor_set_torque("right-wheel", motor, torque);
}

int32_t cvra_encoder_get_left_ext(void *nothing)
{
    (void) nothing;
    uint32_t left_encoder = encoder_get_left();
    left_encoder_value += encoder_tick_diff(left_encoder_prev, left_encoder);
    left_encoder_prev = left_encoder;
    return left_encoder_value;
}

int32_t cvra_encoder_get_right_ext(void *nothing)
{
    (void) nothing;
    uint32_t right_encoder = encoder_get_right();
    right_encoder_value += encoder_tick_diff(right_encoder_prev, right_encoder);
    right_encoder_prev = right_encoder;
    return right_encoder_value;
}
