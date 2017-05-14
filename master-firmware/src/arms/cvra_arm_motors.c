#include <error/error.h>
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


void set_motor_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_set_position(dev->m, dev->direction * position + dev->index);
}

void set_motor_velocity(void* motor, float velocity)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_set_velocity(dev->m, dev->direction * velocity);
}

float get_motor_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    return dev->direction * (motor_driver_get_and_clear_stream_value(dev->m, MOTOR_STREAM_POSITION) - dev->index);
}


void set_wrist_position(void* wrist, float heading, float pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;

    float up = 0.5 * ((heading + dev->heading_index) + (pitch + dev->pitch_index));
    float down = 0.5 * ((heading + dev->heading_index) - (pitch + dev->pitch_index));

    motor_driver_set_position(dev->up, dev->up_direction * up);
    motor_driver_set_position(dev->down, dev->down_direction * down);
}

void set_wrist_velocity(void* wrist, float heading, float pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;

    float up = 0.5 * (heading + pitch);
    float down = 0.5 * (heading - pitch);

    motor_driver_set_velocity(dev->up, dev->up_direction * up);
    motor_driver_set_velocity(dev->down, dev->down_direction * down);
}

void get_wrist_position(void* wrist, float* heading, float* pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;

    float up = dev->up_direction * motor_driver_get_and_clear_stream_value(dev->up, MOTOR_STREAM_POSITION);
    float down = dev->down_direction * motor_driver_get_and_clear_stream_value(dev->down, MOTOR_STREAM_POSITION);

    *heading = up + down - dev->heading_index;
    *pitch = up - down - dev->pitch_index;
}
