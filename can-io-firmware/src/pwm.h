#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>

void pwm_init(uint32_t frequency, uint32_t period);

/* Note: channel is the channel number from datasheet - 1.
 * eg. TIM1_CH2 has channel number 1. */
void pwm_set_duty_cycle(stm32_tim_t *tim, uint8_t channel, uint32_t width);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
