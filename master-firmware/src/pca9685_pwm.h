#ifndef PCA9685_PWM_H
#define PCA9685_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void pca9685_pwm_init(float period_sec);
void pca9685_pwm_output_enable(bool enable);
void pca9685_pwm_set_pulse_width(unsigned int pwm_nb, float pulse_width_sec);
void pca9685_pwm_set_duty_cycle(unsigned int pwm_nb, float duty_cycle);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_PWM_H */
