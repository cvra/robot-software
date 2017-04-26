#include "main.h"
#include "motor_helpers.h"
#include "error/error.h"

// By how many radians we overshoot the index, in order to detect the hysteresis
static const float MOTOR_INDEX_ANGLE_OVERSHOOT = 0.6; // in rad

float motor_wait_for_index(motor_driver_t* motor, float motor_speed)
{
    uint32_t index_count = motor->stream.value_stream_index_update_count;

    NOTICE("Moving axis...");
    motor_driver_set_velocity(motor, motor_speed);

    while (motor->stream.value_stream_index_update_count == index_count) {
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    return motor_driver_get_and_clear_stream_value(motor, MOTOR_STREAM_INDEX);
}

float motor_auto_index(motor_driver_t* motor, int motor_dir, float motor_speed)
{
    float index_rising = motor_wait_for_index(motor, -motor_dir * motor_speed);

    NOTICE("Seen index for first time at %.4f!", index_rising);
#ifndef TESTS
    chThdSleepMilliseconds(1000.f * MOTOR_INDEX_ANGLE_OVERSHOOT / motor_speed);
#endif

    float index_falling = motor_wait_for_index(motor, motor_dir * motor_speed);

    NOTICE("Seen index for second time at %.4f!", index_falling);
#ifndef TESTS
    chThdSleepMilliseconds(1000.f * MOTOR_INDEX_ANGLE_OVERSHOOT / motor_speed);
#endif

    motor_driver_set_velocity(motor, 0);
    float index_avg = 0.5f * (index_rising + index_falling);

    return index_avg;
}
