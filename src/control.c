#include "motor_pwm.h"
#include "control.h"
#include <ch.h>
#include <hal.h>
#include <pid/pid.h>

#define LOW_BATT_TH 9.0f // [V]

#define ADC_MAX         4096
#define ADC_TO_AMPS     0.001611328125f
#define ADC_TO_VOLTS    0.005283043033f

static float motor_current;
static float u_batt;
static float motor_voltage;

static void motor_set_voltage(float u)
{
    motor_voltage = u;
    if (u_batt > LOW_BATT_TH) {
        motor_pwm_enable();
        motor_pwm_set(u / u_batt);
    } else {
        motor_pwm_disable();
    }
}

float control_get_battery_voltage(void)
{
    return u_batt;
}

float control_get_motor_current(void)
{
    return motor_current;
}

float control_get_motor_voltage(void)
{
    return motor_voltage;
}

static THD_FUNCTION(adc_task, arg)
{
    (void)arg;
    chRegSetThreadName("adc read");
    static adcsample_t adc_samples[4];
    static const ADCConversionGroup adcgrpcfg1 = {
        FALSE,                  // circular?
        4,                      // nb channels
        NULL,                   // callback fn
        NULL,                   // error callback fn
        0,                      // CFGR
        0,                      // TR1
        6,                      // CCR : DUAL=regualar,simultaneous
        {ADC_SMPR1_SMP_AN1(0), 0},                          // SMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {ADC_SMPR1_SMP_AN2(0) | ADC_SMPR1_SMP_AN3(0), 0},   // SSMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
    };

    adcStart(&ADCD1, NULL);

    while (1) {
        adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, 1);
        motor_current = motor_current * 0.99
            + (- (adc_samples[1] - ADC_MAX / 2) * ADC_TO_AMPS) * 0.01;
        u_batt = adc_samples[3] * ADC_TO_VOLTS;
    }
    return 0;
}

static THD_FUNCTION(control_loop, arg)
{
    (void)arg;
    chRegSetThreadName("Control Loop");

    static pid_filter_t current_pid;
    pid_init(&current_pid);
    pid_set_gains(&current_pid, 20.f, 15.f, 0.0f);
    pid_set_frequency(&current_pid, 1000.f);

    while (42) {
        float current_setpt = -0.150;     // Amps
        if (u_batt < LOW_BATT_TH) {
            pid_reset_integral(&current_pid);
        }
        motor_set_voltage(pid_process(&current_pid, current_setpt - motor_current));
        chThdSleepMicroseconds(1000000.f / pid_get_frequency(&current_pid));
    }

    return 0;
}

void control_start(void)
{
    static THD_WORKING_AREA(adc_task_wa, 256);
    static THD_WORKING_AREA(control_loop_wa, 256);

    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), LOWPRIO, adc_task, NULL);
    chThdCreateStatic(control_loop_wa, sizeof(control_loop_wa), LOWPRIO, control_loop, NULL);
}

