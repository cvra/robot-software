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
#include "motor_protection.h"
#include "feedback.h"

#include "control.h"

#define LOW_BATT_TH 12.f // [V]


struct feedback_s control_feedback;
motor_protection_t control_motor_protection;


static struct pid_cascade_s ctrl;

static bool motor_enable = true;

#define SETPT_MODE_POS      0
#define SETPT_MODE_VEL      1
#define SETPT_MODE_TORQUE   2
#define SETPT_MODE_TRAJ     3
static int setpt_mode = SETPT_MODE_TORQUE;
static float target_pos = 0; // valid only in position control mode
static float target_vel = 0; // valid only in velocity control mode
static float setpt_pos = 0;
static float setpt_vel = 0;
static float traj_acc = 0; // acceleration of the trajectory mode
static float setpt_torque = 0;
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
    float breaking_dist = vel * vel / 2 / max_acc;  // distance needed to break with max_acc
    float next_error = pos + vel * delta_t + max_acc / 2 / delta_t / delta_t - target_pos;
    float next_error_sign = copysignf(1.0, next_error);

    if (next_error_sign == copysignf(1.0, vel)) {
        if (fabs(next_error) <= breaking_dist) {
            // too close to break (or just close enough)
            return next_error_sign * max_acc;
        } else if (fabs(vel) >= max_vel) {
            // maximal velocity reched -> just cruise
            return 0;
        } else {
            // we can go faster
            return - next_error_sign * max_acc;
        }
    } else {
        // driving away from target position -> turn around
        return - next_error_sign * max_acc;
    }
}

void control_enable(bool en)
{
    motor_enable = en;
}

void control_update_position_setpoint(float pos)
{
    if (setpt_mode == SETPT_MODE_TORQUE) {
        setpt_pos = pos; // todo replace by current position
        setpt_vel = 0; // todo replace by current velocity
    }
    if (setpt_mode == SETPT_MODE_VEL) {
        setpt_pos = pos; // todo see above
    }
    setpt_mode = SETPT_MODE_POS;
    target_pos = pos;
}

void control_update_velocity_setpoint(float vel)
{
    if (setpt_mode == SETPT_MODE_TORQUE) {
        setpt_vel = 0; // todo see above
    }
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

float control_get_vel_ctrl_out(void)
{
    return ctrl.velocity_ctrl_out;
}

float control_get_pos_ctrl_out(void)
{
    return ctrl.position_ctrl_out;
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
    motor_pwm_set(u / u_batt);
}


struct pid_param_s {
    parameter_t kp;
    parameter_t ki;
    parameter_t kd;
    parameter_t i_limit;
};

static void pid_param_declare(struct pid_param_s *p, parameter_namespace_t *ns)
{
    parameter_scalar_declare_with_default(&p->kp, ns, "kp", 0);
    parameter_scalar_declare_with_default(&p->ki, ns, "ki", 0);
    parameter_scalar_declare_with_default(&p->kd, ns, "kd", 0);
    parameter_scalar_declare_with_default(&p->i_limit, ns, "i_limit", INFINITY);
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
    if (parameter_changed(&p->i_limit)) {
        pid_set_integral_limit(ctrl, parameter_scalar_get(&p->i_limit));
    }
}

// control loop parameters
static parameter_namespace_t param_ns_control;
static parameter_t param_loop_freq;
static parameter_t param_low_batt_th;
static parameter_t param_vel_limit;
static parameter_t param_torque_limit;
static parameter_t param_acc_limit;
static parameter_namespace_t param_ns_pos_ctrl;
static parameter_namespace_t param_ns_vel_ctrl;
static parameter_namespace_t param_ns_cur_ctrl;
static struct pid_param_s pos_pid_params;
static struct pid_param_s vel_pid_params;
static struct pid_param_s cur_pid_params;
static parameter_namespace_t param_ns_motor;
static parameter_t param_torque_cst;
static parameter_namespace_t param_ns_thermal;
static parameter_t param_current_gain;
static parameter_t param_max_temp;
static parameter_t param_Rth;
static parameter_t param_Cth;

void control_declare_parameters(void)
{
    parameter_namespace_declare(&param_ns_control, &parameter_root_ns, "control");
    parameter_scalar_declare_with_default(&param_loop_freq, &param_ns_control, "loop_freq", 1000);
    parameter_scalar_declare_with_default(&param_low_batt_th, &param_ns_control, "low_batt_th", LOW_BATT_TH);
    parameter_scalar_declare(&param_vel_limit, &param_ns_control, "velocity_limit");
    parameter_scalar_declare(&param_torque_limit, &param_ns_control, "torque_limit");
    parameter_scalar_declare(&param_acc_limit, &param_ns_control, "acc_limit");
    parameter_namespace_declare(&param_ns_pos_ctrl, &param_ns_control, "position");
    pid_param_declare(&pos_pid_params, &param_ns_pos_ctrl);
    parameter_namespace_declare(&param_ns_vel_ctrl, &param_ns_control, "velocity");
    pid_param_declare(&vel_pid_params, &param_ns_vel_ctrl);
    parameter_namespace_declare(&param_ns_cur_ctrl, &param_ns_control, "current");
    pid_param_declare(&cur_pid_params, &param_ns_cur_ctrl);

    parameter_namespace_declare(&param_ns_motor, &parameter_root_ns, "motor");
    parameter_scalar_declare(&param_torque_cst, &param_ns_motor, "torque_cst");
    parameter_namespace_declare(&param_ns_thermal, &parameter_root_ns, "thermal");
    parameter_scalar_declare(&param_current_gain, &param_ns_thermal, "current_gain");
    parameter_scalar_declare(&param_max_temp, &param_ns_thermal, "max_temp");
    parameter_scalar_declare(&param_Rth, &param_ns_thermal, "Rth");
    parameter_scalar_declare(&param_Cth, &param_ns_thermal, "Cth");
}


static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    pid_init(&ctrl.current_pid);
    pid_init(&ctrl.velocity_pid);
    pid_init(&ctrl.position_pid);
    ctrl.motor_current_constant = 1;
    ctrl.velocity_limit = INFINITY;
    ctrl.torque_limit = INFINITY;
    ctrl.current_limit = INFINITY;

    float acc_max = INFINITY; // acceleration limit in speed / position control
    float low_batt_th = LOW_BATT_TH;

    float t_max = 0; // todo
    float r_th = 0;
    float c_th = 0;
    float current_gain = 0;
    motor_protection_init(&control_motor_protection, t_max, r_th, c_th, current_gain);


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
            if (parameter_changed(&param_low_batt_th)) {
                low_batt_th = parameter_scalar_get(&param_low_batt_th);
            }
            if (parameter_changed(&param_vel_limit)) {
                ctrl.velocity_limit = parameter_scalar_get(&param_vel_limit);
            }
            if (parameter_changed(&param_torque_limit)) {
                ctrl.torque_limit = parameter_scalar_get(&param_torque_limit);
            }
            if (parameter_changed(&param_acc_limit)) {
                acc_max = parameter_scalar_get(&param_acc_limit);
            }
        }
        if (parameter_namespace_contains_changed(&param_ns_motor)) {
            if (parameter_changed(&param_torque_cst)) {
                ctrl.motor_current_constant = parameter_scalar_get(&param_torque_cst);
            }
            if (parameter_changed(&param_current_gain)) {
                current_gain = parameter_scalar_get(&param_current_gain);
            }
            if (parameter_changed(&param_max_temp)) {
                t_max = parameter_scalar_get(&param_max_temp);
            }
            if (parameter_changed(&param_Rth)) {
                r_th = parameter_scalar_get(&param_Rth);
            }
            if (parameter_changed(&param_Cth)) {
                c_th = parameter_scalar_get(&param_Cth);
            }
        }

        float delta_t = control_period_us * 1000000;
        if (!motor_enable || analog_get_battery_voltage() < low_batt_th) {
            pid_reset_integral(&ctrl.current_pid);
            pid_reset_integral(&ctrl.velocity_pid);
            pid_reset_integral(&ctrl.position_pid);
            motor_pwm_disable();
            motor_protection_update(&control_motor_protection, analog_get_motor_current(), delta_t);
        } else {

            // sensor feedback
            control_feedback.input.potentiometer = analog_get_auxiliary();
            control_feedback.input.primary_encoder = encoder_get_primary();
            control_feedback.input.secondary_encoder = encoder_get_secondary();
            control_feedback.input.delta_t = delta_t;

            feedback_compute(&control_feedback);

            ctrl.position = control_feedback.output.position;
            ctrl.velocity = control_feedback.output.velocity;
            ctrl.current = analog_get_motor_current();

            // ctrl.current_limit = motor_protection_update(&control_motor_protection, ctrl.current, delta_t);

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
                setpt_vel += filter_limit_sym(delta_vel, delta_t * acc_max);
                ctrl.velocity_setpt = setpt_vel;
                ctrl.feedforward_torque = 0;

            } else { // setpt_mode == SETPT_MODE_POS
                ctrl.position_control_enabled = true;
                ctrl.velocity_control_enabled = true;
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
            motor_pwm_enable();
        }

        chThdSleepMicroseconds(control_period_us);
    }
    return 0;
}

void control_start()
{
    static THD_WORKING_AREA(control_loop_wa, 256);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), HIGHPRIO, control_loop, NULL);
}

