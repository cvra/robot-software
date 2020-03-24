#include <ch.h>
#include <hal.h>
#include "pwm.h"
#include <error/error.h>

#define TIMER_FREQUENCY (12 * 1000 * 1000)
#define PWM_FREQUENCY (24 * 1000)
#define PWM_PERIOD (TIMER_FREQUENCY / PWM_FREQUENCY)

void pump_init(void)
{
    NOTICE("pump init");
    /* Timer16 channel 1 is pump 0*/
    rccResetAPB2(RCC_APB2RSTR_TIM16RST);
    rccEnableAPB2(RCC_APB2ENR_TIM16EN, false);
    pwm_setup_channels(STM32_TIM16,
                       PWM_CHANNEL(1),
                       STM32_TIMCLK2,
                       TIMER_FREQUENCY,
                       PWM_PERIOD);

    /* Timer17 channel 1 */
    rccResetAPB2(RCC_APB2RSTR_TIM17RST);
    rccEnableAPB2(RCC_APB2ENR_TIM17EN, false);
    pwm_setup_channels(STM32_TIM17,
                       PWM_CHANNEL(1),
                       STM32_TIMCLK2,
                       TIMER_FREQUENCY,
                       PWM_PERIOD);
}

void pump_set_pwm(int index, float duty_cycle)
{
    if (duty_cycle < 0.) {
        duty_cycle = 0.f;
    }

    if (duty_cycle > 1.) {
        duty_cycle = 1.f;
    }

    uint32_t width = (uint32_t)(duty_cycle * PWM_PERIOD);
    switch (index) {
        case 0:
            pwm_set_pulsewidth(STM32_TIM16, 0, width);
            break;
        case 1:
            pwm_set_pulsewidth(STM32_TIM17, 0, width);
            break;
        default:
            ERROR("Invalid pump %d", index);
    }
}

void pump_set_solenoid(int index, int on)
{
    switch (index) {
        case 0:
            palWritePad(GPIOA, GPIOA_SOLENOID0, on);
            break;
        case 1:
            palWritePad(GPIOA, GPIOA_SOLENOID1, on);
            break;
        default:
            ERROR("Invalid solenoid %d", index);
    }
}
