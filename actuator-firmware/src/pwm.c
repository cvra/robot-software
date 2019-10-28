#include <ch.h>
#include <hal.h>
#include "pwm.h"

#define PWM_CHANNEL(x) 1 << (x - 1) // Mask to select channel number, 1 indexed

static void pwm_setup_channel(stm32_tim_t* tim,
                              uint8_t channel_mask,
                              uint32_t clock,
                              uint32_t frequency,
                              uint32_t period)
{
    uint32_t ccer, ccmr1, ccmr2;
    /* Driver re-configuration scenario, it must be stopped first.*/
    tim->CR1 = 0; /* Timer disabled. */
    tim->CCR[0] = 0; /* Comparator 1 disabled. */
    tim->CCR[1] = 0; /* Comparator 2 disabled. */
    tim->CCR[2] = 0; /* Comparator 3 disabled. */
    tim->CCR[3] = 0; /* Comparator 4 disabled. */
    tim->CNT = 0; /* Counter reset to zero. */
    /* Timer configuration.*/
    tim->PSC = (clock / frequency) - 1;
    tim->ARR = period - 1;
    tim->CR2 = 0;
    /* Output enable and polarity setup.*/
    ccer = 0;
    ccmr1 = 0;
    ccmr2 = 0;
    if (channel_mask & (1 << 0)) {
        ccer |= STM32_TIM_CCER_CC1E;
        ccmr1 |= STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE;
    }
    if (channel_mask & (1 << 1)) {
        ccer |= STM32_TIM_CCER_CC2E;
        ccmr1 |= STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;
    }
    if (channel_mask & (1 << 2)) {
        ccer |= STM32_TIM_CCER_CC3E;
        ccmr2 |= STM32_TIM_CCMR2_OC3M(6) | STM32_TIM_CCMR2_OC3PE;
    }
    if (channel_mask & (1 << 3)) {
        ccer |= STM32_TIM_CCER_CC4E;
        ccmr2 |= STM32_TIM_CCMR2_OC4M(6) | STM32_TIM_CCMR2_OC4PE;
    }
    tim->CCER = ccer;
    tim->CCMR1 = ccmr1;
    tim->CCMR2 = ccmr2;
    tim->EGR = STM32_TIM_EGR_UG; /* Update event. */
    tim->SR = 0; /* Clear pending IRQs. */
    tim->DIER = ~STM32_TIM_DIER_IRQ_MASK; /* DMA-related DIER settings. */
    tim->BDTR = STM32_TIM_BDTR_MOE; /* Main output enable. */
    /* Timer configured and started.*/
    tim->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}

/* Note: channel is the channel number from datasheet - 1.
 * eg. TIM1_CH2 has channel number 1. */
void pwm_set_pulsewidth_lld(stm32_tim_t* tim, uint8_t channel, uint32_t width)
{
    tim->CCR[channel] = width;
}

void pwm_set_pulsewidth(enum pwm_channel channel, uint32_t pulsewidth)
{
    if (channel == PWM_CHANNEL_0) {
        pwm_set_pulsewidth_lld(STM32_TIM16, 0, pulsewidth);
    } else if (channel == PWM_CHANNEL_1) {
        pwm_set_pulsewidth_lld(STM32_TIM17, 0, pulsewidth);
    } else if (channel == PWM_CHANNEL_2) {
        pwm_set_pulsewidth_lld(STM32_TIM1, 1, pulsewidth);
    } else if (channel == PWM_CHANNEL_3) {
        pwm_set_pulsewidth_lld(STM32_TIM1, 2, pulsewidth);
    }
}

void pwm_init(uint32_t frequency, uint32_t period)
{
    /* Timer2 channel 2 and 3 */
    rccResetAPB2(RCC_APB2RSTR_TIM1RST);
    rccEnableAPB2(RCC_APB2ENR_TIM1EN, false);
    pwm_setup_channel(STM32_TIM1,
                      PWM_CHANNEL(2) || PWM_CHANNEL(3),
                      STM32_TIMCLK2,
                      frequency,
                      period);

    /* Timer16 channel 1 */
    rccResetAPB2(RCC_APB2RSTR_TIM16RST);
    rccEnableAPB2(RCC_APB2ENR_TIM16EN, false);
    pwm_setup_channel(STM32_TIM16,
                      PWM_CHANNEL(1),
                      STM32_TIMCLK2,
                      frequency,
                      period);

    /* Timer17 channel 1 */
    rccResetAPB2(RCC_APB2RSTR_TIM17RST);
    rccEnableAPB2(RCC_APB2ENR_TIM17EN, false);
    pwm_setup_channel(STM32_TIM17,
                      PWM_CHANNEL(1),
                      STM32_TIMCLK2,
                      frequency,
                      period);
}
