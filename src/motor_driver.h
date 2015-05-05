#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <parameter/parameter.h>

#define MOTOR_ID_MAX_LEN 24

#define MOTOR_CONTROL_MODE_POSITION 1
#define MOTOR_CONTROL_MODE_VELOCITY 2
#define MOTOR_CONTROL_MODE_TORQUE   3
#define MOTOR_CONTROL_MODE_VOLTAGE  4

typedef struct {
    char id[MOTOR_ID_MAX_LEN+1];
    int can_id;
    int control_mode;

} motor_driver_t;


#ifdef __cplusplus
extern "C" {
#endif

void motor_driver_init(motor_driver_t *d,
                       const char *actuator_id,
                       parameter_namespace_t *ns);

// returns a pointer to the stored id
const char *motor_driver_get_id(motor_driver_t *d);

void motor_driver_set_velocity(motor_driver_t *d, float velocity);
void motor_driver_set_position(motor_driver_t *d, float position);
void motor_driver_update_trajectory(motor_driver_t *d, trajectory_t traj);

void motor_driver_set_can_id(motor_driver_t *d, int can_id);

#define CAN_ID_NOT_SET  0xFFFF
int motor_driver_get_can_id(motor_driver_t *d);

parameter_namespace_t *motor_driver_get_parameter_namespace(motor_driver_t *d);

motor_driver_t* motor_driver_list_get_next(motor_driver_t *d);
// returns new head pointer
motor_driver_t* motor_driver_list_add(motor_driver_t *head, motor_driver_t *new_driver);


enum control_mode motor_driver_get_control_mode(motor_driver_t *d);

float motor_driver_get_velocity(motor_driver_t *d);
float motor_driver_get_position(motor_driver_t *d);
void motor_driver_get_trajectory_point(motor_driver_t *d,
                                       unix_timestamp_t timestamp,
                                       float *position,
                                       float *velocity,
                                       float *acceleration,
                                       float *torque);

// sets trajectory buffer allocated by motor manager (ChibiOS block allocater)
void motor_driver_set_traj_buffer(motor_driver_t *d, trajectory_buffer_t *traj_buffer);
// gets the trajectory buffer to free it (or check for its existance)
trajectory_buffer_t *motor_driver_get_traj_buffer(motor_driver_t *d);


/*
 * - Drivers are a linked list allocated by the motor_manager
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