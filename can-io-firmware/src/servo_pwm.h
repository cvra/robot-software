#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

void servo_set(const float pos[4]);
void servo_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_PWM_H */
