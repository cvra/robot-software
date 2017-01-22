#include "motor_manager.h"

/* Moves motor with given speed and waits until a new index is detected */
float motor_wait_for_index(motor_driver_t* motor, float motor_speed);

/* Moves motor forward then backwards to get index from both sides and returns average index */
float motor_auto_index(motor_driver_t* motor, int motor_dir, float motor_speed);
