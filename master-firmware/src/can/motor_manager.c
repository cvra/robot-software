#include <ch.h>
#include <string.h>
#include "motor_manager.h"
#include "config.h"
#include "main.h"
#include <error/error.h>

void motor_manager_init(motor_manager_t* m,
                        motor_driver_t* motor_driver_buffer,
                        uint16_t motor_driver_buffer_len,
                        bus_enumerator_t* bus_enumerator)
{
    memset(motor_driver_buffer, 0,
           motor_driver_buffer_len * sizeof(motor_driver_t));

    m->motor_driver_buffer = motor_driver_buffer;
    m->motor_driver_buffer_len = motor_driver_buffer_len;
    m->bus_enumerator = bus_enumerator;

    m->motor_driver_buffer_nb_elements = 0;
}

motor_driver_t* motor_manager_create_driver(motor_manager_t* m,
                                            const char* actuator_id)
{
    if (bus_enumerator_get_can_id(m->bus_enumerator, actuator_id) != BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
        return NULL;
    }
    NOTICE("create motor %s", actuator_id);

    if (m->motor_driver_buffer_nb_elements < m->motor_driver_buffer_len) {
        motor_driver_t* driver = &m->motor_driver_buffer[m->motor_driver_buffer_nb_elements];

        motor_driver_init(driver,
                          actuator_id,
                          &actuator_config);

        m->motor_driver_buffer_nb_elements++;

        bus_enumerator_add_node(m->bus_enumerator, motor_driver_get_id(driver), (void*)driver);

        return driver;
    } else {
        ERROR("Motor driver memory allocation failed.");
        return NULL;
    }
}

static motor_driver_t* get_driver(motor_manager_t* m, const char* actuator_id)
{
    return (motor_driver_t*)bus_enumerator_get_driver(m->bus_enumerator, actuator_id);
}

void motor_manager_get_list(motor_manager_t* m, motor_driver_t** buffer, uint16_t* length)
{
    *buffer = m->motor_driver_buffer;
    *length = m->motor_driver_buffer_nb_elements;
}

void motor_manager_set_voltage(motor_manager_t* m,
                               const char* actuator_id,
                               float voltage)
{
    motor_driver_t* driver;
    driver = get_driver(m, actuator_id);

    if (driver == NULL) {
        // control error
        return;
    }
    motor_driver_set_voltage(driver, voltage);
}

void motor_manager_set_torque(motor_manager_t* m,
                              const char* actuator_id,
                              float torque)
{
    motor_driver_t* driver;
    driver = get_driver(m, actuator_id);

    if (driver == NULL) {
        // control error
        return;
    }
    motor_driver_set_torque(driver, torque);
}

void motor_manager_set_velocity(motor_manager_t* m,
                                const char* actuator_id,
                                float velocity)
{
    motor_driver_t* driver;
    driver = get_driver(m, actuator_id);

    if (driver == NULL) {
        // control error
        return;
    }
    motor_driver_set_velocity(driver, velocity);
}

void motor_manager_set_position(motor_manager_t* m,
                                const char* actuator_id,
                                float position)
{
    motor_driver_t* driver;
    driver = get_driver(m, actuator_id);

    if (driver == NULL) {
        // control error
        return;
    }
    motor_driver_set_position(driver, position);
}
