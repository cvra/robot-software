#ifndef CVRA_ARM_MOTORS_H
#define CVRA_ARM_MOTORS_H

#include "can/motor_driver.h"
#include "can/motor_manager.h"

typedef struct {
    char id[MOTOR_ID_MAX_LEN_WITH_NUL];
    float direction;
    float index;
} cvra_arm_motor_t;

typedef struct {
    char up[MOTOR_ID_MAX_LEN_WITH_NUL];
    char down[MOTOR_ID_MAX_LEN_WITH_NUL];
    float up_direction;
    float down_direction;

    float heading_ratio; // Heading to motor angle ratio
    float pitch_ratio;   // Pitch to motor angle ratio
    float heading_index; // Heading index, wired on up
    float pitch_index;   // Pitch index, wired on down
} cvra_arm_wrist_t;

motor_driver_t* get_motor_driver(const char* name);

void set_motor_position(void* motor, float position);
void set_motor_velocity(void* motor, float velocity);
float get_motor_position(void* motor);

void set_wrist_position(void* wrist, float heading, float pitch);
void set_wrist_velocity(void* wrist, float heading, float pitch);
void get_wrist_position(void* wrist, float* heading, float* pitch);

#endif
