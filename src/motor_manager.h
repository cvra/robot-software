#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

/*

# The motor manager

- allocates motor drivers
- uses the bus_enumerator to map motor ids to can ids & drivers
- provides an iterator on all active motors for the control loop
- manages trajectory buffers (block allocator) for the drivers
- updates the driver's CAN ID (by asking the bus enumerator)

 */

#include <ch.h>
#include <stdint.h>
#include "motor_driver.h"
#include "trajectories.h"


#define MOTOR_MANAGER_ALLOCATED_TRAJECTORY_LENGTH   100

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    memory_pool_t traj_buffer_pool;
    float *trajectory_buffer;
    uint16_t trajectory_buffer_len;
    motor_driver_t *motor_driver_buffer;
    uint16_t motor_driver_buffer_len;
    uint16_t motor_driver_buffer_nb_elements;
} motor_manager_t;


void motor_manager_init(motor_manager_t *m,
                        float *trajectory_buffer,
                        uint16_t trajectory_buffer_len,
                        motor_driver_t *motor_driver_buffer,
                        uint16_t motor_driver_buffer_len);


// motor_driver_t elements form an array
void motor_manager_get_list(motor_manager_t *m, motor_driver_t **buffer, uint16_t *length);

// setters: get driver, allocate driver if it doesn't exist, panic if allocation fails,
// use setter for driver, (allocate trajectory_buffer for driver if needed)
void motor_manager_set_velocity(motor_manager_t *m,
                                const char *actuator_id,
                                float velocity);

void motor_manager_set_position(motor_manager_t *m,
                                const char *actuator_id,
                                float position);

void motor_manager_execute_trajecory(motor_manager_t *m,
                                     const char *actuator_id,
                                     trajectory_chunk_t *traj);


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MANAGER_H */
