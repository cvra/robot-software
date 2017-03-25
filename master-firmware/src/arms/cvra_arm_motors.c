#include "motor_manager.h"
#include "main.h"

#include "cvra_arm_motors.h"

void set_left_shoulder_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "left-shoulder", dev->direction * position);
}

void set_left_elbow_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "left-elbow", dev->direction * position);
}

void set_left_wrist_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "left-wrist", dev->direction * position);
}


void set_right_shoulder_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "right-shoulder", dev->direction * position);
}

void set_right_elbow_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "right-elbow", dev->direction * position);
}

void set_right_wrist_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "right-wrist", dev->direction * position);
}
