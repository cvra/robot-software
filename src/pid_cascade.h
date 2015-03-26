#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "pid/pid.h"

struct pid_cascade_s {
    // contorllers:
    pid_ctrl_t current_pid;
    pid_ctrl_t velocity_pid;
    pid_ctrl_t position_pid;
    // parameters:
    float motor_current_constant;
    float velocity_limit;
    float torque_limit;
    float current_limit;
    // setpoints:
    bool position_control_enabled;
    bool velocity_control_enabled;
    float position_setpt;
    float velocity_setpt;
    float feedforward_torque;
    // inputs:
    float position;
    float velocity;
    float current;
    // outputs:
    float motor_voltage;
    // PID tuning outputs:
    float position_error;
    float position_ctrl_out;
    float velocity_error;
    float velocity_ctrl_out;
    float current_error;
    float torque;
};


void pid_cascade_control(struct pid_cascade_s *ctrl);


#ifdef __cplusplus
}
#endif

#endif /* PID_CASCADE_H */