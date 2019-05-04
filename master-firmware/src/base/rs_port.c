#include "base/encoder.h"

#include "rs_port.h"

#define MAX_MOTOR_VOLTAGE_SCALE 1000.f

uint32_t left_encoder_prev, right_encoder_prev;
int32_t left_encoder_value, right_encoder_value;

void rs_encoder_init(void)
{
    left_encoder_prev = encoder_get_left();
    right_encoder_prev = encoder_get_right();
    left_encoder_value = left_encoder_prev;
    right_encoder_value = right_encoder_prev;
}

static void motor_set_voltage(const char* id, void* motor, int32_t voltage)
{
    rs_motor_t* dev = (rs_motor_t*)motor;

    float vel = voltage * dev->direction / MAX_MOTOR_VOLTAGE_SCALE;

    motor_manager_set_voltage(dev->m, id, vel);
}

void rs_left_wheel_set_voltage(void* motor, int32_t voltage)
{
    motor_set_voltage("left-wheel", motor, voltage);
}

void rs_right_wheel_set_voltage(void* motor, int32_t voltage)
{
    motor_set_voltage("right-wheel", motor, voltage);
}

int32_t rs_encoder_get_left_ext(void* nothing)
{
    (void)nothing;
    uint32_t left_encoder = encoder_get_left();
    left_encoder_value += encoder_tick_diff(left_encoder_prev, left_encoder);
    left_encoder_prev = left_encoder;
    return left_encoder_value;
}

int32_t rs_encoder_get_right_ext(void* nothing)
{
    (void)nothing;
    uint32_t right_encoder = encoder_get_right();
    right_encoder_value += encoder_tick_diff(right_encoder_prev, right_encoder);
    right_encoder_prev = right_encoder;
    return right_encoder_value;
}
