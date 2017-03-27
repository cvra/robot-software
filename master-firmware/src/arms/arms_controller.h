#ifndef ARMS_CONTROLLER_H
#define ARMS_CONTROLLER_H

#include "scara/scara.h"

#define ARMS_FREQUENCY 10

extern scara_t left_arm;
extern scara_t right_arm;

/* Initialize arms */
void arms_init(void);

/* Auto index a motor given its name */
float arms_motor_auto_index(const char* motor_name, int motor_dir, float motor_speed);

/* Run arms controller thread */
void arms_controller_start(void);

/* Index a whole bunch of motors simultaneously */
void arms_auto_index(char** motor_names, int* motor_dirs, float* motor_speeds, size_t num_motors, float* motor_indexes);

/* Set motor index value */
void arms_set_motor_index(void* motor, float index);

#endif
