#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);

void servo_set(unsigned int servo_nb, float pos);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
