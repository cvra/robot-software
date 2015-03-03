#include <ch.h>
#include <hal.h>
#include <math.h>
#include <pid/pid.h>
#include "motor_pwm.h"
#include "analog.h"
#include "encoder.h"
#include "parameter/parameter.h"
#include "main.h"
#include "pid_cascade.h"

#include "control.h"

#define LOW_BATT_TH 12.f // [V]




static struct pid_cascade_s ctrl;



void control_update_position_setpoint(float pos)
{

}

void control_update_velocity_setpoint(float vel)
{

}

void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque)
{

}

float control_get_motor_voltage(void)
{
    return ctrl.motor_voltage;
}

float control_get_current(void)
{
    return ctrl.current;
}

float control_get_torque(void)
{
    return ctrl.torque;
}

float control_get_velocity(void)
{
    return ctrl.velocity;
}

float control_get_position(void)
{
    return ctrl.position;
}

float control_get_current_error(void)
{
    return ctrl.current_error;
}

float control_get_velocity_error(void)
{
    return ctrl.velocity_error;
}

float control_get_position_error(void)
{
    return ctrl.position_error;
}


static void motor_set_voltage(float u)
{
    float u_batt = analog_get_battery_voltage();
    if (u_batt > LOW_BATT_TH) {
        motor_pwm_enable();
        motor_pwm_set(u / u_batt);
    } else {
        motor_pwm_disable();
    }
}


struct pid_param_s {
    parameter_t kp;
    parameter_t ki;
    parameter_t kd;
};

static void pid_param_declare(struct pid_param_s *p, parameter_namespace_t *ns)
{
    parameter_scalar_declare_with_default(&p->kp, ns, "kp", 0);
    parameter_scalar_declare_with_default(&p->ki, ns, "ki", 0);
    parameter_scalar_declare_with_default(&p->kd, ns, "kd", 0);
}

static void pid_param_update(struct pid_param_s *p, pid_ctrl_t *ctrl)
{
    if (parameter_changed(&p->kp) ||
        parameter_changed(&p->ki) ||
        parameter_changed(&p->kd)) {
        pid_set_gains(ctrl, parameter_scalar_get(&p->kp),
                            parameter_scalar_get(&p->ki),
                            parameter_scalar_get(&p->kd));
        pid_reset_integral(ctrl);
    }
}

// control loop parameters
static parameter_namespace_t param_ns_control;
static parameter_t param_loop_freq;
static parameter_namespace_t param_ns_pos_ctrl;
static parameter_namespace_t param_ns_vel_ctrl;
static parameter_namespace_t param_ns_cur_ctrl;
static struct pid_param_s pos_pid_params;
static struct pid_param_s vel_pid_params;
static struct pid_param_s cur_pid_params;

static void control_declare_parameters(void)
{
    parameter_namespace_declare(&param_ns_control, &parameter_root_ns, "control");
    parameter_scalar_declare_with_default(&param_loop_freq, &param_ns_control, "loop_freq", 1000);
    parameter_namespace_declare(&param_ns_pos_ctrl, &param_ns_control, "position");
    parameter_namespace_declare(&param_ns_vel_ctrl, &param_ns_control, "velocity");
    parameter_namespace_declare(&param_ns_cur_ctrl, &param_ns_control, "current");
    pid_param_declare(&pos_pid_params, &param_ns_pos_ctrl);
    pid_param_declare(&vel_pid_params, &param_ns_vel_ctrl);
    pid_param_declare(&cur_pid_params, &param_ns_cur_ctrl);
}


static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    pid_init(&ctrl.current_pid);
    pid_init(&ctrl.velocity_pid);
    pid_init(&ctrl.position_pid);
    ctrl.motor_current_constant = 0;
    ctrl.velocity_limit = INFINITY;
    ctrl.torque_limit = INFINITY;
    ctrl.current_limit = INFINITY;

    uint32_t control_period_us = 0;
    while (true) {
        // update parameters if they changed
        if (parameter_namespace_contains_changed(&param_ns_control)) {
            if (parameter_namespace_contains_changed(&param_ns_pos_ctrl)) {
                pid_param_update(&pos_pid_params, &ctrl.position_pid);
            }
            if (parameter_namespace_contains_changed(&param_ns_vel_ctrl)) {
                pid_param_update(&vel_pid_params, &ctrl.velocity_pid);
            }
            if (parameter_namespace_contains_changed(&param_ns_cur_ctrl)) {
                pid_param_update(&cur_pid_params, &ctrl.current_pid);
            }
            if (parameter_changed(&param_loop_freq)) {
                float freq = parameter_scalar_get(&param_loop_freq);
                control_period_us = 1000000.f/freq;
                pid_set_frequency(&ctrl.current_pid, freq);
                pid_set_frequency(&ctrl.velocity_pid, freq);
                pid_set_frequency(&ctrl.position_pid, freq);
            }
        }

        if (analog_get_battery_voltage() < LOW_BATT_TH) {
            pid_reset_integral(&ctrl.current_pid);
            pid_reset_integral(&ctrl.velocity_pid);
            pid_reset_integral(&ctrl.position_pid);
        } else {
            ctrl.position = 0;
            ctrl.velocity = 0;
            ctrl.current = 0;
            ctrl.position_control_enabled = false;
            ctrl.position_setpt = 0;
            ctrl.velocity_setpt = 0;
            ctrl.feedforward_torque = 0;

            pid_cascade_control(&ctrl);
            motor_set_voltage(ctrl.motor_voltage);
        }

        chThdSleepMicroseconds(control_period_us);
    }
    return 0;
}

void control_start(void)
{
    control_declare_parameters();

    static THD_WORKING_AREA(control_loop_wa, 256);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), HIGHPRIO, control_loop, NULL);
}

