#ifndef CVRA_MOTOR_H
#define CVRA_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_manager.h"

typedef struct {
    motor_manager_t *m;
    float direction;
} cvra_motor_t;

void cvra_motor_left_wheel_set_torque(void* motor, int32_t torque);
void cvra_motor_right_wheel_set_torque(void* motor, int32_t torque);

void cvra_encoder_init(void);
int32_t cvra_encoder_get_left_ext(void *nothing);
int32_t cvra_encoder_get_right_ext(void *nothing);

#ifdef __cplusplus
}
#endif

#endif /* CVRA_MOTOR_H */
