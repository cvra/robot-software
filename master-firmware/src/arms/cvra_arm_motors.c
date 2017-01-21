#include "motor_manager.h"
#include "main.h"

void set_left_shoulder_position(float position)
{
    motor_manager_set_position(&motor_manager, "left-shoulder", position);
}

void set_left_elbow_position(float position)
{
    motor_manager_set_position(&motor_manager, "left-elbow", position);
}


void set_right_shoulder_position(float position)
{
    motor_manager_set_position(&motor_manager, "right-shoulder", position);
}

void set_right_elbow_position(float position)
{
    motor_manager_set_position(&motor_manager, "right-elbow", position);
}
