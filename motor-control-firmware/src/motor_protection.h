#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float t;
    float t_max;
    float t_ambient;
    float r_th;
    float c_th;
    float current_gain;
} motor_protection_t;

void motor_protection_init(motor_protection_t *p, float t_max, float r_th, float c_th, float current_gain);
float motor_protection_update(motor_protection_t *p, float current, float delta_t);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTECTION_H */
