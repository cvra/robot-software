#include <ch.h>
#include <hal.h>
#include <math.h>
#include <pid/pid.h>
#include "motor_pwm.h"
#include "analog.h"
#include "encoder.h"
#include <parameter/parameter.h>
#include "main.h"
#include "pid_cascade.h"
#include <timestamp/timestamp.h>
#include <filter/basic.h>
#include "motor_protection.h"
#include "feedback.h"
#include "setpoint.h"

#include "control.h"

#define LOW_BATT_TH 5.f // [V]
#define DEFAULT_CTRL_TIMEOUT 0.3f // [s]

struct pid_param_s {
    parameter_t kp;
    parameter_t ki;
    parameter_t kd;
    parameter_t i_limit;
};

struct feedback_s control_feedback;
motor_protection_t control_motor_protection;

binary_semaphore_t setpoint_interpolation_lock;
static setpoint_interpolator_t setpoint_interpolation;
static struct pid_cascade_s ctrl;

/** Control loop parameters */
static struct {
    parameter_namespace_t ns;
    parameter_t low_batt_th;
    struct {
        parameter_t vel;
        parameter_t acc;
        parameter_t torque;
    } limits;

    struct {
        parameter_namespace_t ns;
        struct pid_param_s pid;
    } pos, vel, cur;

    parameter_t mode;
} control_params;

/** Motor-specific parameters */
static struct {
    parameter_namespace_t ns;
    parameter_t torque_cst;
    parameter_t current_offset;
} motor_params;

/* Thermal protection parameters */
static struct {
    parameter_namespace_t ns;
    parameter_t current_gain;
    parameter_t Rth;
    parameter_t Cth;
    parameter_t max_temp;
} thermal_params;

/** Encoders specific parameters. */
static struct {
    parameter_namespace_t ns;
    struct {
        parameter_namespace_t ns;
        parameter_t p;
        parameter_t q;
        parameter_t ticks_per_rev;
    } primary, secondary;
} encoder_params;

static struct {
    parameter_namespace_t ns;
    parameter_t gain;
    parameter_t zero;
} potentiometer_params;

static struct {
    parameter_namespace_t ns;
    parameter_t phase;
} rpm_params;

static timestamp_t last_setpoint_update;

static float low_batt_th = LOW_BATT_TH;
static float ctrl_timeout = DEFAULT_CTRL_TIMEOUT;

static bool control_request_termination = false;
static bool control_running = false;

void control_update_position_setpoint(float pos)
{
    float current_pos = ctrl.position;
    float current_vel = ctrl.velocity;
    chBSemWait(&setpoint_interpolation_lock);
    setpoint_update_position(&setpoint_interpolation, pos, current_pos, current_vel, ctrl.periodic_actuator);
    chBSemSignal(&setpoint_interpolation_lock);
    last_setpoint_update = timestamp_get();

    ctrl.position_setpoint = pos;
}

void control_update_velocity_setpoint(float vel)
{
    float current_vel = ctrl.velocity;
    chBSemWait(&setpoint_interpolation_lock);
    setpoint_update_velocity(&setpoint_interpolation, vel, current_vel);
    chBSemSignal(&setpoint_interpolation_lock);
    last_setpoint_update = timestamp_get();

    ctrl.velocity_setpoint = vel;
}

void control_update_torque_setpoint(float torque)
{
    chBSemWait(&setpoint_interpolation_lock);
    setpoint_update_torque(&setpoint_interpolation, torque);
    chBSemSignal(&setpoint_interpolation_lock);
    last_setpoint_update = timestamp_get();

    ctrl.current_setpoint = torque * ctrl.motor_current_constant;
}

void control_update_trajectory_setpoint(float pos, float vel, float acc, float torque, timestamp_t ts)
{
    chBSemWait(&setpoint_interpolation_lock);
    setpoint_update_trajectory(&setpoint_interpolation, pos, vel, acc, torque, ts);
    chBSemSignal(&setpoint_interpolation_lock);
    last_setpoint_update = timestamp_get();

    ctrl.position_setpoint = pos;
    ctrl.velocity_setpoint = vel;
    ctrl.current_setpoint = torque * ctrl.motor_current_constant;
}

void control_update_voltage_setpoint(float voltage)
{
    chBSemWait(&setpoint_interpolation_lock);
    setpoint_update_voltage(&setpoint_interpolation, voltage);
    chBSemSignal(&setpoint_interpolation_lock);
    last_setpoint_update = timestamp_get();
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

float control_get_current_setpoint(void)
{
    return ctrl.current_setpoint;
}

float control_get_velocity_setpoint(void)
{
    return ctrl.velocity_setpoint;
}

float control_get_position_setpoint(void)
{
    return ctrl.position_setpoint;
}

static void set_motor_voltage(float u)
{
    float u_batt = analog_get_battery_voltage();
    motor_pwm_set(u / u_batt);
}

static void pid_param_declare(struct pid_param_s* p, parameter_namespace_t* ns)
{
    parameter_scalar_declare_with_default(&p->kp, ns, "kp", 0);
    parameter_scalar_declare_with_default(&p->ki, ns, "ki", 0);
    parameter_scalar_declare_with_default(&p->kd, ns, "kd", 0);
    parameter_scalar_declare_with_default(&p->i_limit, ns, "i_limit", INFINITY);
}

static void pid_param_update(struct pid_param_s* p, pid_ctrl_t* ctrl)
{
    if (parameter_changed(&p->kp) || parameter_changed(&p->ki) || parameter_changed(&p->kd)) {
        pid_set_gains(ctrl, parameter_scalar_get(&p->kp),
                      parameter_scalar_get(&p->ki),
                      parameter_scalar_get(&p->kd));
        pid_reset_integral(ctrl);
    }
    if (parameter_changed(&p->i_limit)) {
        pid_set_integral_limit(ctrl, parameter_scalar_get(&p->i_limit));
    }
}

static void declare_parameters(void)
{
    /* Control parameters */
    parameter_namespace_declare(&control_params.ns, &parameter_root_ns, "control");
    parameter_scalar_declare_with_default(&control_params.low_batt_th, &control_params.ns, "low_batt_th", LOW_BATT_TH);
    parameter_scalar_declare_with_default(&control_params.limits.vel, &control_params.ns, "velocity_limit", INFINITY);
    parameter_scalar_declare_with_default(&control_params.limits.torque, &control_params.ns, "torque_limit", INFINITY);
    parameter_scalar_declare_with_default(&control_params.limits.acc, &control_params.ns, "acceleration_limit", INFINITY);

    parameter_namespace_declare(&control_params.pos.ns, &control_params.ns, "position");
    pid_param_declare(&control_params.pos.pid, &control_params.pos.ns);
    parameter_namespace_declare(&control_params.vel.ns, &control_params.ns, "velocity");
    pid_param_declare(&control_params.vel.pid, &control_params.vel.ns);
    parameter_namespace_declare(&control_params.cur.ns, &control_params.ns, "current");
    pid_param_declare(&control_params.cur.pid, &control_params.cur.ns);
    parameter_integer_declare_with_default(&control_params.mode, &control_params.ns, "mode", 0);

    /* Motor parameters. */
    parameter_namespace_declare(&motor_params.ns, &parameter_root_ns, "motor");
    parameter_scalar_declare_with_default(&motor_params.torque_cst, &motor_params.ns, "torque_cst", 1.);
    parameter_scalar_declare_with_default(&motor_params.current_offset, &motor_params.ns, "current_offset", 0.);

    /* Thermal */
    parameter_namespace_declare(&thermal_params.ns, &parameter_root_ns, "thermal");
    parameter_scalar_declare_with_default(&thermal_params.current_gain, &thermal_params.ns, "current_gain", 1.);
    parameter_scalar_declare_with_default(&thermal_params.max_temp, &thermal_params.ns, "max_temp", INFINITY);
    parameter_scalar_declare_with_default(&thermal_params.Rth, &thermal_params.ns, "Rth", 1.);
    parameter_scalar_declare_with_default(&thermal_params.Cth, &thermal_params.ns, "Cth", INFINITY);

    /* Encoders */
    parameter_namespace_declare(&encoder_params.ns, &parameter_root_ns, "encoders");

    /* Primary */
    parameter_namespace_declare(&encoder_params.primary.ns, &encoder_params.ns, "primary");
    parameter_integer_declare_with_default(&encoder_params.primary.p, &encoder_params.primary.ns, "p", 1);
    parameter_integer_declare_with_default(&encoder_params.primary.q, &encoder_params.primary.ns, "q", 1);
    parameter_integer_declare_with_default(&encoder_params.primary.ticks_per_rev, &encoder_params.primary.ns, "ticks_per_rev", 1024);

    /* secondary */
    parameter_namespace_declare(&encoder_params.secondary.ns, &encoder_params.ns, "secondary");
    parameter_integer_declare_with_default(&encoder_params.secondary.p, &encoder_params.secondary.ns, "p", 1);
    parameter_integer_declare_with_default(&encoder_params.secondary.q, &encoder_params.secondary.ns, "q", 1);
    parameter_integer_declare_with_default(&encoder_params.secondary.ticks_per_rev, &encoder_params.secondary.ns, "ticks_per_rev", 1024);

    /* potentiometer */
    parameter_namespace_declare(&potentiometer_params.ns, &parameter_root_ns, "potentiometer");
    parameter_scalar_declare_with_default(&potentiometer_params.gain, &potentiometer_params.ns, "gain", 1.);
    parameter_scalar_declare_with_default(&potentiometer_params.zero, &potentiometer_params.ns, "zero", 0.);

    parameter_namespace_declare(&rpm_params.ns, &parameter_root_ns, "rpm");
    parameter_scalar_declare_with_default(&rpm_params.phase, &rpm_params.ns, "phase", 0.);
}

static void update_parameters(void);

void control_init(void)
{
    declare_parameters();

    ctrl.motor_current_constant = 1;
    pid_init(&ctrl.current_pid);
    pid_init(&ctrl.velocity_pid);
    pid_init(&ctrl.position_pid);
    pid_set_frequency(&ctrl.current_pid, ANALOG_CONVERSION_FREQUENCY);
    pid_set_frequency(&ctrl.velocity_pid, ANALOG_CONVERSION_FREQUENCY);
    pid_set_frequency(&ctrl.position_pid, ANALOG_CONVERSION_FREQUENCY);

    setpoint_init(&setpoint_interpolation);
    chBSemObjectInit(&setpoint_interpolation_lock, false);

    control_feedback.output.position = 0;
    control_feedback.output.velocity = 0;
    control_feedback.primary_encoder.accumulator = 0;
    control_feedback.secondary_encoder.accumulator = 0;

    last_setpoint_update = timestamp_get();

    control_feedback.primary_encoder.transmission_p = parameter_integer_get(&encoder_params.primary.p);
    control_feedback.primary_encoder.transmission_q = parameter_integer_get(&encoder_params.primary.q);

    ctrl.torque_limit = parameter_scalar_get(&control_params.limits.torque);
    ctrl.current_limit = ctrl.torque_limit / parameter_scalar_get(&motor_params.torque_cst);
    ctrl.velocity_limit = parameter_scalar_get(&control_params.limits.vel);

    float transmission = (float)control_feedback.primary_encoder.transmission_p / control_feedback.primary_encoder.transmission_q;
    ctrl.motor_current_constant = 1.f / (parameter_scalar_get(&motor_params.torque_cst) * transmission);

    update_parameters();
}

static void update_parameters(void)
{
    control_feedback.input_selection = parameter_integer_get(&control_params.mode);

    control_feedback.primary_encoder.transmission_p = parameter_integer_get(&encoder_params.primary.p);
    control_feedback.primary_encoder.transmission_q = parameter_integer_get(&encoder_params.primary.q);
    control_feedback.primary_encoder.ticks_per_rev = parameter_integer_get(&encoder_params.primary.ticks_per_rev);

    control_feedback.secondary_encoder.transmission_p = parameter_integer_get(&encoder_params.secondary.p);
    control_feedback.secondary_encoder.transmission_q = parameter_integer_get(&encoder_params.secondary.q);
    control_feedback.secondary_encoder.ticks_per_rev = parameter_integer_get(&encoder_params.secondary.ticks_per_rev);

    control_feedback.potentiometer.gain = parameter_scalar_get(&potentiometer_params.gain);
    control_feedback.potentiometer.zero = parameter_scalar_get(&potentiometer_params.zero);

    control_feedback.rpm.phase = parameter_scalar_get(&rpm_params.phase);

    if (parameter_namespace_contains_changed(&control_params.ns)) {
        if (parameter_namespace_contains_changed(&control_params.pos.ns)) {
            pid_param_update(&control_params.pos.pid, &ctrl.position_pid);
        }
        if (parameter_namespace_contains_changed(&control_params.vel.ns)) {
            pid_param_update(&control_params.vel.pid, &ctrl.velocity_pid);
        }
        if (parameter_namespace_contains_changed(&control_params.cur.ns)) {
            pid_param_update(&control_params.cur.pid, &ctrl.current_pid);
        }
        if (parameter_changed(&control_params.low_batt_th)) {
            low_batt_th = parameter_scalar_get(&control_params.low_batt_th);
        }
        if (parameter_changed(&control_params.limits.vel)) {
            ctrl.velocity_limit = parameter_scalar_get(&control_params.limits.vel);
            chBSemWait(&setpoint_interpolation_lock);
            setpoint_set_velocity_limit(&setpoint_interpolation, ctrl.velocity_limit);
            chBSemSignal(&setpoint_interpolation_lock);
        }
        if (parameter_changed(&control_params.limits.torque)) {
            ctrl.torque_limit = parameter_scalar_get(&control_params.limits.torque);
            ctrl.current_limit = ctrl.torque_limit / parameter_scalar_get(&motor_params.torque_cst);
        }
        if (parameter_changed(&control_params.limits.acc)) {
            chBSemWait(&setpoint_interpolation_lock);
            setpoint_set_acceleration_limit(&setpoint_interpolation, parameter_scalar_get(&control_params.limits.acc));
            chBSemSignal(&setpoint_interpolation_lock);
        }
    }
    if (parameter_namespace_contains_changed(&motor_params.ns)) {
        if (parameter_changed(&motor_params.torque_cst)) {
            float transmission = (float)control_feedback.primary_encoder.transmission_p / control_feedback.primary_encoder.transmission_q;
            ctrl.motor_current_constant = 1.f / (parameter_scalar_get(&motor_params.torque_cst) * transmission);
        }
        if (parameter_changed(&motor_params.current_offset)) {
            ctrl.motor_current_offset = parameter_scalar_get(&motor_params.current_offset);
        }
    }
    if (parameter_namespace_contains_changed(&thermal_params.ns)) {
        motor_protection_init(&control_motor_protection,
                              parameter_scalar_get(&thermal_params.max_temp),
                              parameter_scalar_get(&thermal_params.Rth),
                              parameter_scalar_get(&thermal_params.Cth),
                              parameter_scalar_get(&thermal_params.current_gain));
    }
}

#define CONTROL_WAKEUP_EVENT 1

static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    control_feedback.primary_encoder.previous = encoder_get_primary();
    control_feedback.secondary_encoder.previous = encoder_get_secondary();

    static event_listener_t analog_event_listener;
    chEvtRegisterMaskWithFlags(&analog_event, &analog_event_listener,
                               (eventmask_t)CONTROL_WAKEUP_EVENT,
                               (eventflags_t)ANALOG_EVENT_CONVERSION_DONE);

    const float delta_t = 1 / (float)ANALOG_CONVERSION_FREQUENCY;
    while (!control_request_termination) {
        update_parameters();

        // sensor feedback
        control_feedback.input.potentiometer = analog_get_auxiliary();
        control_feedback.input.primary_encoder = encoder_get_primary();
        control_feedback.input.secondary_encoder = encoder_get_secondary();
        control_feedback.input.delta_t = delta_t;

        feedback_compute(&control_feedback);

        ctrl.periodic_actuator = control_feedback.output.actuator_is_periodic;
        ctrl.position = control_feedback.output.position;
        ctrl.velocity = ctrl.velocity * 0.9 + control_feedback.output.velocity * 0.1;
        ctrl.current = analog_get_motor_current() - ctrl.motor_current_offset;
        ctrl.torque = ctrl.current / ctrl.motor_current_constant;
        // ctrl.current_limit = motor_protection_update(&control_motor_protection, ctrl.current, delta_t);

        timestamp_t now = timestamp_get();
        if (analog_get_battery_voltage() < low_batt_th
            || timestamp_duration_s(last_setpoint_update, now) > ctrl_timeout) {
            pid_reset_integral(&ctrl.current_pid);
            pid_reset_integral(&ctrl.velocity_pid);
            pid_reset_integral(&ctrl.position_pid);
            set_motor_voltage(0);
        } else {
            // setpoints
            chBSemWait(&setpoint_interpolation_lock);
            setpoint_compute(&setpoint_interpolation, &ctrl.setpts, delta_t);
            chBSemSignal(&setpoint_interpolation_lock);

            // run control step
            pid_cascade_control(&ctrl);

            if (setpoint_interpolation.setpt_mode == SETPT_MODE_VOLT) {
                set_motor_voltage(setpoint_interpolation.setpt_voltage);
            } else {
                set_motor_voltage(ctrl.motor_voltage);
            }
        }

        chEvtWaitAny(CONTROL_WAKEUP_EVENT);
        chEvtGetAndClearFlags(&analog_event_listener);
    }

    set_motor_voltage(0);
    chEvtUnregister(&analog_event, &analog_event_listener);
    control_running = false;
}

void control_start(void)
{
    control_running = true;
    static THD_WORKING_AREA(control_loop_wa, 256);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), HIGHPRIO, control_loop, NULL);
}

void control_stop(void)
{
    control_request_termination = true;
    while (control_running) {
        chThdSleepMilliseconds(1);
    }
    control_request_termination = false;
}
