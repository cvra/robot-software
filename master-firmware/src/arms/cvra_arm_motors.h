#ifndef CVRA_ARM_MOTORS_H
#define CVRA_ARM_MOTORS_H

#include "can/motor_driver.h"
#include "can/motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char id[MOTOR_ID_MAX_LEN_WITH_NUL];
    float direction;
    float index;
} cvra_arm_motor_t;

motor_driver_t* get_motor_driver(const char* name);

void set_motor_position(void* motor, float position);
void set_motor_velocity(void* motor, float velocity);
void set_motor_voltage(void* motor, float voltage);
float get_motor_position(void* motor);

#ifdef __cplusplus
}
#endif

#endif
