#include <ch.h>
#include <hal.h>
#include <chprintf.h>

BaseSequentialStream* stdout;

#define PWM_PERIOD      2880

static PWMConfig pwm_cfg = {
    72000000,
    PWM_PERIOD,           // 25kHz
    NULL,
    // activate channel 1 and 2
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,                  // TIMx_CR2 value
    0                   // TIMx_DIER value
};

static void pwm_setup(void)
{
    pwmStart(&PWMD1, &pwm_cfg);
}



static THD_WORKING_AREA(adc_task_wa, 128);
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
        {0, 0},                 // SMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {0, 0},                 // SSMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
    };

    while (1) {
        adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, 1);
        chprintf(stdout, "%d %d %d %d\n", adc_samples[0], adc_samples[1], adc_samples[2], adc_samples[3]);
        // chThdSleepMilliseconds(80);
    }
    return 0;
}


int main(void) {
    halInit();
    chSysInit();

    pwm_setup();

    pwmEnableChannel(&PWMD1, 0, 0.1 * PWM_PERIOD);
    pwmEnableChannel(&PWMD1, 1, 0.0 * PWM_PERIOD);

    palSetPad(GPIOA, GPIOA_MOTOR_EN_A);
    palSetPad(GPIOA, GPIOA_MOTOR_EN_B);

    adcStart(&ADCD1, NULL);

    sdStart(&SD3, NULL);
    stdout = (BaseSequentialStream*)&SD3;

    chprintf(stdout, "boot\n");

    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), LOWPRIO, adc_task, NULL);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
