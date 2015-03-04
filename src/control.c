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
#include "timestamp/timestamp.h"
#include "filter/basic.h"

#include "control.h"

#define LOW_BATT_TH 12.f // [V]



static struct pid_cascade_s ctrl;

#define SETPT_MODE_POS      0
#define SETPT_MODE_VEL      1
#define SETPT_MODE_TORQUE   2
#define SETPT_MODE_TRAJ     3
static int setpt_mode;
static float target_pos; // valid only in position control mode
static float target_vel; // valid only in velocity control mode
static float setpt_pos;
static float setpt_vel;
static float traj_acc; // acceleration of the trajectory mode
static float setpt_torque;
static timestamp_t setpt_ts; // timestamp of the last setpoint update (traj. mode)


static float pos_setpt_interpolation(float pos, float vel, float acc, float delta_t)
{
    return pos + vel * delta_t + acc / 2 * delta_t * delta_t;
}

static float vel_setpt_interpolation(float vel, float acc, float delta_t)
{
    return vel + acc * delta_t;
}

// returns acceleration to be applied for the next delta_t
static float vel_ramp(float pos, float vel, float target_pos, float delta_t, float max_vel, float max_acc)
{
    (void)pos;
    (void)vel;
    (void)target_pos;
    (void)delta_t;
    (void)max_vel;
    (void)max_acc;
    return 0; // todo
}

void control_update_position_setpoint(float pos)
{
    setpt_mode = SETPT_MODE_POS;
    target_pos = pos;
}

void control_update_velocity_setpoint(float vel)
{
    setpt_mode = SETPT_MODE_VEL;
    target_vel = vel;
}

void control_update_torque_setpoint(float torque)
{
    setpt_mode = SETPT_MODE_TORQUE;
    setpt_torque = torque;
}

void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque, timestamp_t ts)
{
    setpt_mode = SETPT_MODE_TRAJ;
    setpt_pos = pos;
    setpt_vel = vel;
    traj_acc = acc;
    setpt_torque = torque;
    setpt_ts = ts;
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

    float acc_max = INFINITY; // acceleration limit in speed / position control

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

            // sensor feedback
            ctrl.position = 0; // todo
            ctrl.velocity = 0; // todo
            ctrl.current = analog_get_motor_current();

            // setpoints
            if (setpt_mode == SETPT_MODE_TRAJ) {
                timestamp_t now = timestamp_get();
                float delta_t = timestamp_duration_s(setpt_ts, now);
                ctrl.position_control_enabled = true;
                ctrl.velocity_control_enabled = true;
                ctrl.position_setpt = pos_setpt_interpolation(setpt_pos, setpt_vel, traj_acc, delta_t);
                ctrl.velocity_setpt = vel_setpt_interpolation(setpt_vel, traj_acc, delta_t);
                ctrl.feedforward_torque = setpt_torque;

            } else if (setpt_mode == SETPT_MODE_TORQUE) {
                ctrl.position_control_enabled = false;
                ctrl.velocity_control_enabled = false;
                ctrl.feedforward_torque = setpt_torque;

            } else if (setpt_mode == SETPT_MODE_VEL) {
                ctrl.position_control_enabled = false;
                ctrl.velocity_control_enabled = true;
                float delta_vel = target_vel - setpt_vel;
                float delta_t = control_period_us * 1000000;
                ctrl.velocity_setpt = filter_limit_sym(delta_vel, delta_t * acc_max);
                ctrl.feedforward_torque = 0;

            } else { // SETPT_MODE_POS
                ctrl.position_control_enabled = true;
                ctrl.velocity_control_enabled = true;
                float delta_t = control_period_us * 1000000;
                float acc = vel_ramp(setpt_pos, setpt_vel, target_pos, delta_t, ctrl.velocity_limit, acc_max);
                float pos = pos_setpt_interpolation(setpt_pos, setpt_vel, acc, delta_t);
                float vel = vel_setpt_interpolation(setpt_vel, acc, delta_t);
                ctrl.position_setpt = setpt_pos = pos;;
                ctrl.velocity_setpt = setpt_vel = vel;
                ctrl.feedforward_torque = 0;
            }

            // run control step
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

