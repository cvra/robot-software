#include <ch.h>
#include <hal.h>
#include <pid/pid.h>
#include "motor_pwm.h"
#include "analog.h"
#include "encoder.h"

#include "control.h"

#define LOW_BATT_TH 9.0f // [V]


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

typedef struct zero_crossing_filter_s {
    int sign;
    int zero_crossing_countdown;
    int zero_crossing_limit;
} zero_crossing_filter_t;

static int float_sign(float f)
{
    if (f >= 0) {
        return 1;
    } else {
        return -1;
    }
}

void zero_crossing_filter_init(zero_crossing_filter_t *f, int limit)
{
    f->sign = 1;
    f->zero_crossing_countdown = 0;
    f->zero_crossing_limit = limit;
}

float zero_crossing_filter_apply(zero_crossing_filter_t *f, float in)
{
    if (f->zero_crossing_countdown > 0) {
        f->zero_crossing_countdown--;
    }
    if (f->sign == float_sign(in)) { // sign didn't change
        return in;
    }
    // sign changed
    if (f->zero_crossing_countdown == 0) {
        f->zero_crossing_countdown = f->zero_crossing_limit;
        f->sign = - f->sign;
        return in;
    } else {
        return 0;
    }
}

static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    static pid_filter_t current_pid;
    pid_init(&current_pid);
    pid_set_gains(&current_pid, 20.f, 15.f, 0.0f);
    pid_set_frequency(&current_pid, 1000.f);

    static zero_crossing_filter_t zero_crossing_filter;
    zero_crossing_filter_init(&zero_crossing_filter, 10);

    while (42) {
        float current_setpt = -0.150;     // Amps
        if (analog_get_battery_voltage() < LOW_BATT_TH) {
            pid_reset_integral(&current_pid);
        }
        float pid_out = pid_process(&current_pid, current_setpt - analog_get_motor_current());
        motor_set_voltage(zero_crossing_filter_apply(&zero_crossing_filter, pid_out));
        chThdSleepMicroseconds(1000000.f / pid_get_frequency(&current_pid));
    }

    return 0;
}

void control_start(void)
{
    static THD_WORKING_AREA(control_loop_wa, 256);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), HIGHPRIO, control_loop, NULL);
}

