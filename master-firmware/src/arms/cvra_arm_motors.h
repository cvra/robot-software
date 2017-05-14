#ifndef CVRA_ARM_MOTORS_H
#define CVRA_ARM_MOTORS_H

typedef struct {
    motor_manager_t *m;
    float direction;
    float index;
} cvra_arm_motor_t;


void set_left_z_position(void* motor, float position);
void set_left_shoulder_position(void* motor, float position);
void set_left_elbow_position(void* motor, float position);
void set_left_wrist_position(void* motor, float position);

void set_left_shoulder_velocity(void* motor, float velocity);
void set_left_elbow_velocity(void* motor, float velocity);
void set_left_wrist_velocity(void* motor, float velocity);

void set_right_z_position(void* motor, float position);
void set_right_shoulder_position(void* motor, float position);
void set_right_elbow_position(void* motor, float position);
void set_right_wrist_position(void* motor, float position);

void set_right_shoulder_velocity(void* motor, float velocity);
void set_right_elbow_velocity(void* motor, float velocity);
void set_right_wrist_velocity(void* motor, float velocity);

float get_left_z_position(void* motor);
float get_left_shoulder_position(void* motor);
float get_left_elbow_position(void* motor);
float get_left_wrist_position(void* motor);

float get_right_z_position(void* motor);
float get_right_shoulder_position(void* motor);
float get_right_elbow_position(void* motor);
float get_right_wrist_position(void* motor);

#endif
