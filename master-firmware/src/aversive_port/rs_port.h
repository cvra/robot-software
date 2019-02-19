#ifndef AVERSIVE_RS_PORT_H
#define AVERSIVE_RS_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can/motor_manager.h"

typedef struct {
    motor_manager_t* m;
    float direction;
} rs_motor_t;

void rs_left_wheel_set_torque(void* motor, int32_t torque);
void rs_right_wheel_set_torque(void* motor, int32_t torque);

void rs_encoder_init(void);
int32_t rs_encoder_get_left_ext(void* nothing);
int32_t rs_encoder_get_right_ext(void* nothing);

#ifdef __cplusplus
}
#endif

#endif /* AVERSIVE_RS_PORT_H */
