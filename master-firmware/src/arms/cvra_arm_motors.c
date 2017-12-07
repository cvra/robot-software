#include <math.h>
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

void set_motor_voltage(void* motor, float voltage)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* driver = get_motor_driver(dev->id);
    motor_driver_set_voltage(driver, voltage);
}

float get_motor_position(void* motor)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    motor_driver_t* driver = get_motor_driver(dev->id);
    return dev->direction * (motor_driver_get_and_clear_stream_value(driver, MOTOR_STREAM_POSITION) - dev->index);
}
