#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>

/// Mask to select channel number, 1 indexed
#define PWM_CHANNEL(x) (1 << (x - 1))

/** Low level init of PWM channels for a timer.
 *
 * @param [in] tim Address of the timer to use, example STM32_TIM1.
 * @param [in] channel_mask Which channels to enable, OR'd together. You can
 * use PWM_CHANNEL(n) to enable channel N.
 * @param [in] clock_frequency The frequency of the clock running this
 * particular timer peripheral. You can use macros such as STM32_TIMCLK2 to
 * compute it from the RCC configuration.
 * @param [in] timer_frequency The frequency at which the timer should count.
 * @param [in] timer_period The timer period.
 */
void pwm_setup_channels(stm32_tim_t* tim,
                        uint8_t channel_mask,
                        uint32_t clock_frequency,
                        uint32_t timer_frequency,
                        uint32_t timer_period);

/** Sets the pulse width for that particular timer and channel.
 *
 * Note that the channel number is 0-indexed here, while in the datasheet it is
 * 1-indexed.
 */
void pwm_set_pulsewidth(stm32_tim_t* tim, uint8_t channel, uint32_t width);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
