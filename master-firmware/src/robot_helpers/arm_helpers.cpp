#include <ch.h>

#include <algorithm>
#include <array>
#include <math.h>

#include <error/error.h>

#include "manipulator/kinematics.h"
#include "motor_manager.h"
#include "motor_helpers.h"
#include "config.h"
#include "main.h"

#include "arm_helpers.h"

static motor_driver_t* get_motor_driver(const char* name)
{
    motor_driver_t* motor = (motor_driver_t*)bus_enumerator_get_driver(motor_manager.bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}

static void set_index_stream_frequency(const char* motor, float freq)
{
    motor_driver_t* driver = get_motor_driver(motor);

    if (parameter_defined(&(driver->config.index_stream))) {
        parameter_scalar_set(&(driver->config.index_stream), freq);
    } else {
        ERROR("Undefined motor %s", motor);
    }
}

void arm_motors_index(const char** motors, const float* references, const float* directions, const float* speeds, float* offsets)
{
    /* Enable index stream over CAN */
    for (size_t i = 0; i < 3; i++) {
        set_index_stream_frequency(motors[i], 100);
    }
    chThdSleepSeconds(2); // wait for motors to start streaming

    offsets[0] = motor_auto_index(motors[0], 1.f * directions[0], speeds[0]);
    offsets[1] = motor_auto_index(motors[1], -1.f * directions[1], speeds[1]);
    offsets[2] = motor_auto_index(motors[2], 1.f * directions[2], speeds[2]);

    /* Disable index stream over CAN */
    for (size_t i = 0; i < 3; i++) {
        motor_manager_set_torque(&motor_manager, motors[i], 0);
        set_index_stream_frequency(motors[i], 0);
    }

    arm_compute_offsets(references, directions, offsets);
}

void arm_compute_offsets(const float* ref, const float* directions, float* offsets)
{
    std::array<float, 3> references;
    std::copy_n(ref, 3, std::begin(references));

    // axis are decoupled
    // references = manipulator::axes_decouple(references);

    // We don't index at angle 0
    offsets[0] = directions[0] * offsets[0] - references[0];
    offsets[1] = directions[1] * offsets[1] - references[1];
    offsets[2] = directions[2] * offsets[2] - references[2];
}

void arm_manual_index(manipulator_side_t side)
{
    float offsets[3];
    float references[3];

    if (side == RIGHT) {
        offsets[0] = motor_get_position("right-theta-1");
        offsets[1] = motor_get_position("right-theta-2");
        offsets[2] = motor_get_position("right-theta-3");
        references[0] = config_get_scalar("master/arms/right/homing/q1");
        references[1] = config_get_scalar("master/arms/right/homing/q2");
        references[2] = config_get_scalar("master/arms/right/homing/q3");
        float right_directions[3] = {-1, -1, 1};
        arm_compute_offsets(references, right_directions, offsets);

        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q1"), offsets[0]);
        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q2"), offsets[1]);
        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q3"), offsets[2]);
    } else if (side == LEFT) {
        offsets[0] = motor_get_position("left-theta-1");
        offsets[1] = motor_get_position("left-theta-2");
        offsets[2] = motor_get_position("left-theta-3");
        references[0] = config_get_scalar("master/arms/left/homing/q1");
        references[1] = config_get_scalar("master/arms/left/homing/q2");
        references[2] = config_get_scalar("master/arms/left/homing/q3");
        float left_directions[3] = {1, 1, -1};
        arm_compute_offsets(references, left_directions, offsets);

        parameter_scalar_set(PARAMETER("master/arms/left/offsets/q1"), offsets[0]);
        parameter_scalar_set(PARAMETER("master/arms/left/offsets/q2"), offsets[1]);
        parameter_scalar_set(PARAMETER("master/arms/left/offsets/q3"), offsets[2]);
    }
}

void arm_turn_off(manipulator_side_t side)
{
    if (side == RIGHT) {
        motor_manager_set_torque(&motor_manager, "right-theta-1", 0);
        motor_manager_set_torque(&motor_manager, "right-theta-2", 0);
        motor_manager_set_torque(&motor_manager, "right-theta-3", 0);
    } else if (side == LEFT) {
        motor_manager_set_torque(&motor_manager, "left-theta-1", 0);
        motor_manager_set_torque(&motor_manager, "left-theta-2", 0);
        motor_manager_set_torque(&motor_manager, "left-theta-3", 0);
    }
}
