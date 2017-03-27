#ifndef CVRA_ARM_MOTORS_H
#define CVRA_ARM_MOTORS_H

typedef struct {
    motor_manager_t *m;
    float direction;
    float index;
} cvra_arm_motor_t;


void set_left_shoulder_position(void* motor, float position);
void set_left_elbow_position(void* motor, float position);

void set_right_shoulder_position(void* motor, float position);
void set_right_elbow_position(void* motor, float position);

float get_left_shoulder_position(void* motor);
float get_left_elbow_position(void* motor);

float get_right_shoulder_position(void* motor);
float get_right_elbow_position(void* motor);

#endif
