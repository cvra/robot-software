#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <pid/pid.h>
#include "setpoint.h"

struct pid_cascade_s {
    // contorllers:
    pid_ctrl_t current_pid;
    pid_ctrl_t velocity_pid;
    pid_ctrl_t position_pid;
    // parameters:
    float motor_current_constant;
    float motor_current_offset; // constant offset introduced by wiring flaws
    float velocity_limit;
    float torque_limit;
    float current_limit;
    // setpoints:
    struct setpoint_s setpts;
    // inputs:
    bool periodic_actuator;
    float position;
    float velocity;
    float current;
    // outputs:
    float motor_voltage;
    // PID tuning outputs:
    float position_setpoint;
    float position_error;
    float position_ctrl_out;
    float velocity_setpoint;
    float velocity_error;
    float velocity_ctrl_out;
    float current_setpoint;
    float current_error;
    float torque;
};

// todo this should not be here
float periodic_error(float err);

void pid_cascade_control(struct pid_cascade_s* ctrl);

#ifdef __cplusplus
}
#endif

#endif /* PID_CASCADE_H */
