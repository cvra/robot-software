#include <ch.h>
#include <hal.h>
#include <pid/pid.h>
#include "motor_pwm.h"
#include "analog.h"
#include "encoder.h"
#include "parameter/parameter.h"
#include "main.h"

#include "control.h"

#define LOW_BATT_TH 12.f // [V]

static float _current_setpt;
static float _velocity_setpt;

void control_set_current(float c)
{
    _current_setpt = c;
}

void control_set_velocity(float v)
{
    _velocity_setpt = v;
}

static float motor_voltage;

static void motor_set_voltage(float u)
{
    motor_voltage = u;
    float u_batt = analog_get_battery_voltage();
    if (u_batt > LOW_BATT_TH) {
        motor_pwm_enable();
        motor_pwm_set(u / u_batt);
    } else {
        motor_pwm_disable();
    }
}

float control_get_motor_voltage(void)
{
    return motor_voltage;
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

static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    static pid_ctrl_t current_pid;
    static pid_ctrl_t velocity_pid;
    static pid_ctrl_t position_pid;

    pid_init(&current_pid);
    pid_init(&velocity_pid);
    pid_init(&position_pid);

    float motor_current_constant = 1;

    uint32_t control_period_us = 0;
    while (42) {
        // update parameters if they changed
        if (parameter_namespace_contains_changed(&param_ns_control)) {
            if (parameter_namespace_contains_changed(&param_ns_pos_ctrl)) {
                pid_param_update(&pos_pid_params, &position_pid);
            }
            if (parameter_namespace_contains_changed(&param_ns_vel_ctrl)) {
                pid_param_update(&vel_pid_params, &velocity_pid);
            }
            if (parameter_namespace_contains_changed(&param_ns_cur_ctrl)) {
                pid_param_update(&cur_pid_params, &current_pid);
            }
            if (parameter_changed(&param_loop_freq)) {
                float freq = parameter_scalar_get(&param_loop_freq);
                control_period_us = 1000000.f/freq;
                pid_set_frequency(&current_pid, freq);
                pid_set_frequency(&velocity_pid, freq);
                pid_set_frequency(&position_pid, freq);
            }
        }

        // setpoint interpolation
        bool pos_control_enabled = false;
        float position_setpt = 0;
        float velocity_setpt = 100;
        float feedforward_torque = 0;

        // feedback input
        float position = 0;
        float velocity = encoder_get_speed();
        float current = analog_get_motor_current();

        // position control
        float pos_ctrl_vel;
        if (pos_control_enabled) {
            pos_ctrl_vel = pid_process(&position_pid, position - position_setpt);
        } else {
            pid_reset_integral(&position_pid);
            pos_ctrl_vel = 0;
        }

        // velocity control
        velocity_setpt = velocity_setpt + pos_ctrl_vel;
        float vel_ctrl_torque = pid_process(&velocity_pid, velocity - velocity_setpt);

        // torque control
        float torque_setpt = vel_ctrl_torque + feedforward_torque;
        float current_setpt = torque_setpt * motor_current_constant;
        float motor_voltage = pid_process(&current_pid, current - current_setpt);

        motor_set_voltage(motor_voltage);

        if (analog_get_battery_voltage() < LOW_BATT_TH) {
            pid_reset_integral(&current_pid);
            pid_reset_integral(&velocity_pid);
            pid_reset_integral(&position_pid);
        }

        chThdSleepMicroseconds(control_period_us);
    }

    return 0;
}

void control_start(void)
{
    parameter_namespace_declare(&param_ns_control, &parameter_root_ns, "control");
    parameter_scalar_declare_with_default(&param_loop_freq, &param_ns_control, "loop_freq", 1000);
    parameter_namespace_declare(&param_ns_pos_ctrl, &param_ns_control, "position");
    parameter_namespace_declare(&param_ns_vel_ctrl, &param_ns_control, "velocity");
    parameter_namespace_declare(&param_ns_cur_ctrl, &param_ns_control, "current");
    pid_param_declare(&pos_pid_params, &param_ns_pos_ctrl);
    pid_param_declare(&vel_pid_params, &param_ns_vel_ctrl);
    pid_param_declare(&cur_pid_params, &param_ns_cur_ctrl);

    static THD_WORKING_AREA(control_loop_wa, 256);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), HIGHPRIO, control_loop, NULL);
}

