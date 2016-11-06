#include "pid_cascade.h"
#include "math.h"
#include <filter/basic.h>

float periodic_error(float err)
{
    err = fmodf(err, 2*M_PI);
    if (err > M_PI) {
        return err - 2*M_PI;
    }
    if (err < -M_PI) {
        return err + 2*M_PI;
    }
    return err;
}

void pid_cascade_control(struct pid_cascade_s *ctrl)
{
    // position control
    float pos_ctrl_vel;
    if (ctrl->setpts.position_control_enabled) {
        ctrl->position_setpoint = ctrl->setpts.position_setpt;
        float position_error = ctrl->position - ctrl->setpts.position_setpt;
        if (ctrl->periodic_actuator) {
            position_error = periodic_error(position_error);
        }
        ctrl->position_error = position_error;
        pos_ctrl_vel = pid_process(&ctrl->position_pid, ctrl->position_error);
        ctrl->position_ctrl_out = pos_ctrl_vel;
    } else {
        pid_reset_integral(&ctrl->position_pid);
        pos_ctrl_vel = 0;
    }

    // velocity control
    float vel_ctrl_torque;
    if (ctrl->setpts.velocity_control_enabled) {
        float velocity_setpt = ctrl->setpts.velocity_setpt + pos_ctrl_vel;
        velocity_setpt = filter_limit_sym(velocity_setpt, ctrl->velocity_limit);
        ctrl->velocity_setpoint = velocity_setpt;
        ctrl->velocity_error = ctrl->velocity - velocity_setpt;
        vel_ctrl_torque = pid_process(&ctrl->velocity_pid, ctrl->velocity_error);
        ctrl->velocity_ctrl_out = vel_ctrl_torque;
    } else {
        pid_reset_integral(&ctrl->velocity_pid);
        vel_ctrl_torque = 0;
    }

    // torque control
    float torque_setpt = vel_ctrl_torque + ctrl->setpts.feedforward_torque;
    torque_setpt = filter_limit_sym(torque_setpt, ctrl->torque_limit);
    float current_setpt = torque_setpt * ctrl->motor_current_constant;
    current_setpt = filter_limit_sym(current_setpt, ctrl->current_limit);
    ctrl->current_setpoint = current_setpt;
    ctrl->current_error = ctrl->current - current_setpt;
    ctrl->motor_voltage = pid_process(&ctrl->current_pid, ctrl->current_error);
}

