#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif


void motor_pwm_setup(void);

/*
 * dc : duty cycle between -1 and +1, negative for reverse direction
 */
void motor_pwm_set(float dc);

/*
 * drive the motor voltage
 */
void motor_pwm_enable(void);

/*
 * don't drive the motor (floating)
 */
void motor_pwm_disable(void);

/*
 * trigger charge pump recharge cycle (must be called every 2ms)
 */
void motor_pwm_trigger_update_from_isr(bool recharge);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */