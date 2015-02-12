#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif


void motor_pwm_setup(void);

void motor_pwm_set(float dc);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */