#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>

enum pwm_channel {
    PWM_CHANNEL_0, // Timer 16 channel 0
    PWM_CHANNEL_1, // Timer 17 channel 0
    PWM_CHANNEL_2, // Timer 1 channel 1
    PWM_CHANNEL_3, // Timer 1 channel 2
};

void pwm_init(uint32_t frequency, uint32_t period);

/* Set given pulse width in clock ticks to the corresponding PWM channel */
void pwm_set_pulsewidth(enum pwm_channel channel, uint32_t pulsewidth);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
