#include "ch.h"
#include "hal.h"

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

int main(void) {
    halInit();
    chSysInit();

    pwm_setup();

    pwmEnableChannel(&PWMD1, 0, 0.1 * PWM_PERIOD);
    pwmEnableChannel(&PWMD1, 1, 0.0 * PWM_PERIOD);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
