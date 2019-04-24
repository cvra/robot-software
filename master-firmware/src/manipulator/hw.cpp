#include "manipulator/hw.h"

#include <error/error.h>

#include "main.h"
#include "robot_helpers/motor_helpers.h"

namespace manipulator {
Angles System::measure_feedback() const
{
    Angles angles;
    for (size_t i = 0; i < 3; i++) {
        angles[i] = directions[i] * motor_get_position(motors[i]) - offsets[i];
    }

    // axis are decoupled
    angles = axes_couple(angles);

    return angles;
}

void System::apply_input(const Angles& angles)
{
    Angles target;
    for (size_t i = 0; i < 3; i++) {
        target[i] = angles[i] + offsets[i];
    }

    // axis are decoupled
    target = axes_decouple(target);

    for (size_t i = 0; i < 3; i++) {
        target[i] *= directions[i];
        motor_manager_set_position(&motor_manager, motors[i], target[i]);
    }

    last_raw = angles;
    last = target;
}
} // namespace manipulator
