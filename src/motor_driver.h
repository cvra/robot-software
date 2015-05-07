#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <ch.h>
#include <parameter/parameter.h>
#include "unix_timestamp.h"
#include "trajectories.h"

#define MOTOR_ID_MAX_LEN 24

#define MOTOR_CONTROL_MODE_DISABLED     0
#define MOTOR_CONTROL_MODE_POSITION     1
#define MOTOR_CONTROL_MODE_VELOCITY     2
#define MOTOR_CONTROL_MODE_TORQUE       3
#define MOTOR_CONTROL_MODE_VOLTAGE      4
#define MOTOR_CONTROL_MODE_TRAJECTORY   5


struct pid_parameter_s {
    parameter_namespace_t root;
    parameter_t kp;
    parameter_t ki;
    parameter_t kd;
    parameter_t ilimit;
};

typedef struct {
    char id[MOTOR_ID_MAX_LEN+1];
    int can_id;
    binary_semaphore_t lock;
    memory_pool_t *traj_buffer_pool;

    int control_mode;
    union {
        float position;
        float velocity;
        float torque;
        float voltage;
        trajectory_t *trajectory;
    } setpt;

    struct {
        parameter_namespace_t root;
        parameter_namespace_t pid_root;
        struct pid_parameter_s position_pid;
        struct pid_parameter_s velocity_pid;
        struct pid_parameter_s current_pid;
    } config;

} motor_driver_t;


#ifdef __cplusplus
extern "C" {
#endif

// initialize a new motor driver
// - creates a parameter namespace actuator_id for the new driver in the
//   namespace ns
// - the actuator id is stored internally (copied)
void motor_driver_init(motor_driver_t *d,
                       const char *actuator_id,
                       parameter_namespace_t *ns,
                       memory_pool_t *traj_buffer_pool);

// returns a pointer to the stored id string
const char *motor_driver_get_id(motor_driver_t *d);

void motor_driver_set_position(motor_driver_t *d, float position);
void motor_driver_set_velocity(motor_driver_t *d, float velocity);
void motor_driver_set_torque(motor_driver_t *d, float torque);
void motor_driver_set_voltage(motor_driver_t *d, float voltage);
void motor_driver_update_trajectory(motor_driver_t *d, trajectory_chunk_t *traj);
void motor_driver_disable(motor_driver_t *d);

#define CAN_ID_NOT_SET  0xFFFF
int motor_driver_get_can_id(motor_driver_t *d);
void motor_driver_set_can_id(motor_driver_t *d, int can_id);

void motor_driver_lock(motor_driver_t *d);
void motor_driver_unlock(motor_driver_t *d);

// the get control mode and the corresponding setpoint must be called in a
// motor_driver_lock/unlock critical section
int motor_driver_get_control_mode(motor_driver_t *d);

float motor_driver_get_position_setpt(motor_driver_t *d);
float motor_driver_get_velocity_setpt(motor_driver_t *d);
float motor_driver_get_torque_setpt(motor_driver_t *d);
float motor_driver_get_voltage_setpt(motor_driver_t *d);
void motor_driver_get_trajectory_point(motor_driver_t *d,
                                       unix_timestamp_t timestamp,
                                       float *position,
                                       float *velocity,
                                       float *acceleration,
                                       float *torque);

/*
 * - The setpoints are exclusively changed by the motor_manager as well
 * - The control module iterates over the linked list and sends the setpoints to
 *   the corresponding node via UAVCAN
 * - Parameter
 * - C++ with direct uavcan calls?
 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */