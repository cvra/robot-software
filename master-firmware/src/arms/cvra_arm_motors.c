#include <error/error.h>
#include "main.h"

#include "cvra_arm_motors.h"

motor_driver_t* get_motor_driver(const char* name)
{
    motor_driver_t* motor = bus_enumerator_get_driver(motor_manager.bus_enumerator, name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", name);
    }
    return motor;
}


void set_motor_position(void* motor, float position)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* driver = get_motor_driver(dev->id);
    motor_driver_set_position(driver, dev->direction * position + dev->index);
}

void set_motor_velocity(void* motor, float velocity)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* driver = get_motor_driver(dev->id);
    motor_driver_set_velocity(driver, dev->direction * velocity);
}

float get_motor_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* driver = get_motor_driver(dev->id);
    return dev->direction * (motor_driver_get_and_clear_stream_value(driver, MOTOR_STREAM_POSITION) - dev->index);
}


void set_wrist_position(void* wrist, float heading, float pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;
    motor_driver_t* driver_up = get_motor_driver(dev->up);
    motor_driver_t* driver_down = get_motor_driver(dev->down);

    float up = 0.5 * ((heading + dev->heading_index) + (pitch + dev->pitch_index));
    float down = 0.5 * ((heading + dev->heading_index) - (pitch + dev->pitch_index));

    motor_driver_set_position(driver_up, dev->up_direction * up);
    motor_driver_set_position(driver_down, dev->down_direction * down);
}

void set_wrist_velocity(void* wrist, float heading, float pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;
    motor_driver_t* driver_up = get_motor_driver(dev->up);
    motor_driver_t* driver_down = get_motor_driver(dev->down);

    float up = 0.5 * (heading + pitch);
    float down = 0.5 * (heading - pitch);

    motor_driver_set_velocity(driver_up, dev->up_direction * up);
    motor_driver_set_velocity(driver_down, dev->down_direction * down);
}

void get_wrist_position(void* wrist, float* heading, float* pitch)
{
    cvra_arm_wrist_t *dev = (cvra_arm_wrist_t*)wrist;
    motor_driver_t* driver_up = get_motor_driver(dev->up);
    motor_driver_t* driver_down = get_motor_driver(dev->down);

    float up = dev->up_direction * motor_driver_get_and_clear_stream_value(driver_up, MOTOR_STREAM_POSITION);
    float down = dev->down_direction * motor_driver_get_and_clear_stream_value(driver_down, MOTOR_STREAM_POSITION);

    *heading = up + down - dev->heading_index;
    *pitch = up - down - dev->pitch_index;
}
