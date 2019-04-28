#ifndef MOTOR_HELPERS_H
#define MOTOR_HELPERS_H

#include "motor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Moves motor with given torque and waits until a new index is detected */
float motor_wait_for_index(motor_driver_t* motor, float torque);

/* Moves motor forward then backwards to get index from both sides and returns average index */
float motor_auto_index_sym(motor_driver_t* motor, int motor_dir, float torque);

/* Moves motor forward to get index and returns the position of index */
float motor_auto_index(const char* motor_name, int motor_dir, float torque);

/* Read motor feedback */
float motor_get_position(const char* name);
float motor_get_current(const char* name);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HELPERS_H */
