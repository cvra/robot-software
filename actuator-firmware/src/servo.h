#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);
void servo_start(void);
void servo_set(const float pos[4], const float vel[4], const float acc[4]);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
