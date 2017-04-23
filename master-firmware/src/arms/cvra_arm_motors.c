#include <error/error.h>
#include "motor_manager.h"
#include "main.h"

#include "cvra_arm_motors.h"

motor_driver_t* get_motor_driver(motor_manager_t* manager, const char* name)
{
    motor_driver_t* motor = bus_enumerator_get_driver(manager->bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}


void set_left_z_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "left-z", dev->direction * position + dev->index);
}

void set_left_shoulder_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "left-shoulder", dev->direction * position);
}

void set_left_elbow_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "left-elbow", dev->direction * position);
}

void set_left_wrist_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "left-wrist", dev->direction * position);
}


void set_right_z_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_position(dev->m, "right-z", dev->direction * position + dev->index);
}

void set_right_shoulder_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "right-shoulder", dev->direction * position);
}

void set_right_elbow_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "right-elbow", dev->direction * position);
}

void set_right_wrist_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_manager_set_velocity(dev->m, "right-wrist", dev->direction * position);
}


float get_left_z_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "left-z");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_left_shoulder_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "left-shoulder");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_left_elbow_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "left-elbow");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_left_wrist_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "left-wrist");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}


float get_right_z_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "right-z");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_right_shoulder_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "right-shoulder");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_right_elbow_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "right-elbow");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}

float get_right_wrist_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* motord = get_motor_driver(dev->m, "right-wrist");

    return dev->direction * (motor_driver_get_and_clear_stream_value(motord, MOTOR_STREAM_POSITION) - dev->index);
}
