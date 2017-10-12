#ifndef CVRA_ARM_MOTORS_H
#define CVRA_ARM_MOTORS_H

#include "can/motor_driver.h"
#include "can/motor_manager.h"

typedef struct {
    char id[MOTOR_ID_MAX_LEN_WITH_NUL];
    float direction;
    float index;
} cvra_arm_motor_t;

motor_driver_t* get_motor_driver(const char* name);

void set_motor_position(void* motor, float position);
void set_motor_velocity(void* motor, float velocity);
float get_motor_position(void* motor);

#endif
