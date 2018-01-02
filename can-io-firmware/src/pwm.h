#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>

#define SERVO_PWM_TIMER_FREQ    1000000 // 1MHz
#define SERVO_PWM_PERIOD        20000   // 20 ms period

void pwm_init(void);

/* Note: channel is the channel number from datasheet - 1.
 * eg. TIM1_CH2 has channel number 1. */
void pwm_set_duty_cycle(stm32_tim_t *tim, uint8_t channel, uint32_t width);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
