#include "manipulator/hw.h"

#include <error/error.h>

#include "main.h"

namespace {
motor_driver_t* get_motor_driver(const char* name)
{
    motor_driver_t* motor = (motor_driver_t*)bus_enumerator_get_driver(motor_manager.bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}

float read_motor_position(const char* name)
{
    return motor_driver_get_and_clear_stream_value(get_motor_driver(name), MOTOR_STREAM_POSITION);
}
} // namespace

namespace manipulator {
Angles System::measure_feedback() const
{
    Angles angles;
    for (size_t i = 0; i < 3; i++) {
        angles[i] = directions[i] * read_motor_position(motors[i]) - offsets[i];
    }
    return angles;
}

void System::apply_input(const Angles& angles)
{
    for (size_t i = 0; i < 3; i++) {
        motor_driver_set_position(get_motor_driver(motors[i]), directions[i] * angles[i] + offsets[i]);
    }
}
} // namespace manipulator
