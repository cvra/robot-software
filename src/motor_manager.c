#include <ch.h>
#include "motor_manager.h"
#include "bus_enumerator.h"
#include "config.h"
#include "uavcan_node.h"


void motor_manager_init(motor_manager_t *m,
                        float *trajectory_buffer,
                        uint16_t trajectory_buffer_len,
                        motor_driver_t *motor_driver_buffer,
                        uint16_t motor_driver_buffer_len)
{
    m->trajectory_buffer = trajectory_buffer;
    m->trajectory_buffer_len = trajectory_buffer_len;
    m->motor_driver_buffer = motor_driver_buffer;
    m->motor_driver_buffer_len = motor_driver_buffer_len;

    m->motor_driver_buffer_nb_elements = 0;

    chPoolObjectInit(&m->traj_buffer_pool,
                     sizeof(float) * MOTOR_MANAGER_ALLOCATED_TRAJECTORY_LENGTH,
                     NULL);

    chPoolLoadArray(&m->traj_buffer_pool, m->trajectory_buffer, m->trajectory_buffer_len);
}

static motor_driver_t *create_driver(motor_manager_t *m,
                                             const char *actuator_id)
{
    if (m->motor_driver_buffer_nb_elements < m->motor_driver_buffer_len) {
        motor_driver_t *driver = &m->motor_driver_buffer[m->motor_driver_buffer_nb_elements];

        motor_driver_init(driver,
                          actuator_id,
                          &motor_config,
                          &m->traj_buffer_pool);

        m->motor_driver_buffer_nb_elements++;

        bus_enumerator_add_node(&bus_enumerator, motor_driver_get_id(driver), (void*)driver);

        return driver;
    } else {
        return NULL;
    }
}

static motor_driver_t *get_driver(const char *actuator_id)
{
    return (motor_driver_t*)bus_enumerator_get_driver(&bus_enumerator, actuator_id);
}

void motor_manager_get_list(motor_manager_t *m, motor_driver_t **buffer, uint16_t *length)
{
    *buffer = m->motor_driver_buffer;
    *length = m->motor_driver_buffer_nb_elements;
}

void motor_manager_set_velocity(motor_manager_t *m,
                                const char *actuator_id,
                                float velocity)
{
    motor_driver_t *driver;
    driver = get_driver(actuator_id);

    if (driver == NULL) {
        driver = create_driver(m, actuator_id);
        if (driver == NULL) {
            chSysHalt("Motor driver memory allocation failed.");
        }

    }
    motor_driver_set_velocity(driver, velocity);
}

void motor_manager_set_position(motor_manager_t *m,
                                const char *actuator_id,
                                float position)
{
    motor_driver_t *driver;
    driver = get_driver(actuator_id);
    if (driver == NULL) {
        driver =  create_driver(m, actuator_id);
        if (driver == NULL) {
            chSysHalt("Motor driver memory allocation failed.");
        }
    }
    motor_driver_set_position(driver, position);
}

void motor_manager_execute_trajecory(motor_manager_t *m,
                                     const char *actuator_id,
                                     trajectory_chunk_t *traj)
{
    motor_driver_t *driver;
    driver = get_driver(actuator_id);
    if (driver == NULL) {
        driver = create_driver(m, actuator_id);
        if (driver == NULL) {
            chSysHalt("Motor driver memory allocation failed.");
        }
    }
    motor_driver_update_trajectory(driver, traj);
}
