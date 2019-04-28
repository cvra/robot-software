#include <error/error.h>

#include "motor_helpers.h"
#include "main.h"

// By how many radians we overshoot the index, in order to detect the hysteresis
static const float MOTOR_INDEX_ANGLE_OVERSHOOT = 0.6; // in rad

static motor_driver_t* get_motor_driver(const char* name)
{
    motor_driver_t* motor = (motor_driver_t*)bus_enumerator_get_driver(motor_manager.bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}

float motor_wait_for_index(motor_driver_t* motor, float torque)
{
    uint32_t index_count = motor->stream.value_stream_index_update_count;

    NOTICE("Moving axis...");
    motor_driver_set_torque(motor, torque);

    while (motor->stream.value_stream_index_update_count == index_count) {
#ifndef TESTS
        chThdSleepMilliseconds(10);
#endif
    }

    chThdSleepMilliseconds(100);

    return motor_driver_get_and_clear_stream_value(motor, MOTOR_STREAM_INDEX);
}

float motor_auto_index_sym(motor_driver_t* motor, int motor_dir, float torque)
{
    float index_rising = motor_wait_for_index(motor, -motor_dir * torque);

    NOTICE("Seen index for first time at %.4f!", index_rising);
#ifndef TESTS
    chThdSleepMilliseconds(1000.f * MOTOR_INDEX_ANGLE_OVERSHOOT / torque);
#endif

    float index_falling = motor_wait_for_index(motor, motor_dir * torque);

    NOTICE("Seen index for second time at %.4f!", index_falling);
#ifndef TESTS
    chThdSleepMilliseconds(1000.f * MOTOR_INDEX_ANGLE_OVERSHOOT / torque);
#endif

    motor_driver_set_velocity(motor, 0);
    float index_avg = 0.5f * (index_rising + index_falling);

    return index_avg;
}

float motor_auto_index(const char* motor_name, int motor_dir, float torque)
{
    // move until index is found
    float index = motor_wait_for_index(get_motor_driver(motor_name), motor_dir * torque);

    NOTICE("Seen index at %.4f!", index);

    return index;
}

float motor_get_position(const char* name)
{
    return motor_driver_get_and_clear_stream_value(get_motor_driver(name), MOTOR_STREAM_POSITION);
}

float motor_get_current(const char* name)
{
    return motor_driver_get_and_clear_stream_value(get_motor_driver(name), MOTOR_STREAM_CURRENT);
}
