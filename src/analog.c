#include <ch.h>
#include <hal.h>
#include "motor_pwm.h" // to trigger charge pump recharge cycle
#include "analog.h"

#define ADC_MAX         4096
#define ADC_TO_AMPS     0.001611328125f // 3.3/4096/(0.01*50)
#define ADC_TO_VOLTS    0.005281575521f // 3.3/4096/(18/(100+18))

#define ADC_NB_CHANNELS 4
#define DMA_BUFFER_SIZE (243*2)         // dual buffer of 243 (see adc timing below)

#define PWM_RECHARGE_COUNTDOWN_RELOAD 4 // 2kHz / 500Hz (500Hz = recharge freq.)
#define IGNORE_NB_SAMPLES_WHEN_RECHARGING 111 // 486kHz sampling freq -> 19.5 samples / pwm period -> ignore first 3 periods + something because of LP

event_source_t analog_event;

static int32_t motor_current_accumulator=0;
static int32_t motor_current_nb_samples=1;
static int32_t battery_voltage;
static int32_t aux_in;


float analog_get_battery_voltage(void)
{
    return (float)battery_voltage * ADC_TO_VOLTS;
}

float analog_get_motor_current(void)
{
    chSysLock();
    int32_t accu = motor_current_accumulator;
    int32_t nb = motor_current_nb_samples;
    chSysUnlock();
    return -(((float)accu / nb) - ADC_MAX / 2) * ADC_TO_AMPS;
}

float analog_get_auxiliary(void)
{
    return (float)aux_in / (ADC_MAX * 2);
}

static void adc_callback(ADCDriver *adcp, adcsample_t *adc_samples, size_t n)
{
    (void)adcp;

    static int pwm_charge_pump_recharge_countdown = 0;
    bool ignore_first_samples = false;

    if (pwm_charge_pump_recharge_countdown == 0) {
        pwm_charge_pump_recharge_countdown = PWM_RECHARGE_COUNTDOWN_RELOAD;
        motor_pwm_trigger_update_from_isr(true);
    } else {
        motor_pwm_trigger_update_from_isr(false);
    }
    if (pwm_charge_pump_recharge_countdown == PWM_RECHARGE_COUNTDOWN_RELOAD - 1) {
        // previous call triggered a recharge, ignore first samples
        ignore_first_samples = true;
    }
    pwm_charge_pump_recharge_countdown--;

    int i, nb_samples;
    if (ignore_first_samples) {
        i = IGNORE_NB_SAMPLES_WHEN_RECHARGING * ADC_NB_CHANNELS;
        nb_samples = n - IGNORE_NB_SAMPLES_WHEN_RECHARGING;
    } else {
        i = 0;
        nb_samples = n;
    }

    uint32_t accumulator = 0;
    for (; i < (int)(n * ADC_NB_CHANNELS); i += ADC_NB_CHANNELS) {
        accumulator += adc_samples[i + 1];
    }
    chSysLockFromISR();
    motor_current_accumulator = accumulator;
    motor_current_nb_samples = nb_samples;
    chSysUnlockFromISR();

    battery_voltage = adc_samples[3];
    aux_in = adc_samples[0] + adc_samples[2];

    chSysLockFromISR();
    chEvtBroadcastFlagsI(&analog_event, ANALOG_EVENT_CONVERSION_DONE);
    chSysUnlockFromISR();
}

static THD_FUNCTION(adc_task, arg)
{
    (void)arg;
    chRegSetThreadName("adc read");
    static adcsample_t adc_samples[ADC_NB_CHANNELS * DMA_BUFFER_SIZE];
    static const ADCConversionGroup adcgrpcfg1 = {
        TRUE,                   // circular
        ADC_NB_CHANNELS,        // nb channels
        adc_callback,           // callback fn
        NULL,                   // error callback fn
        ADC_CFGR_CONT,          // CFGR
        0,                      // TR1
        6,                      // CCR : DUAL=regular,simultaneous
        /* ADC timing
         * 61.5 sampling + 12.5 conversion ADC cycles (72MHz)
         * sample frequency -> ~973kHz
         * with 2 samples per conversion -> 486kHz sample frequency
         *  243 samples per buffer -> 2.002kHz
         */
        {ADC_SMPR1_SMP_AN1(5), 0}, // SMPRx : 61.5 sampling cycles
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {ADC_SMPR1_SMP_AN2(5) | ADC_SMPR1_SMP_AN3(5), 0},   // SSMPRx : sample time maximum -> ~973kHz
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
        // Expected sampling frequence is 973kHz / 2 = 486kHz
    };

    adcStart(&ADCD1, NULL);

    adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, DMA_BUFFER_SIZE); // should never return
}

void analog_init(void)
{
    static THD_WORKING_AREA(adc_task_wa, 256);
    chEvtObjectInit(&analog_event);
    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), HIGHPRIO, adc_task, NULL);
}
