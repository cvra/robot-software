#ifndef ARMS_CONTROLLER_H
#define ARMS_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "scara/scara.h"
#include "hand/hand.h"
#include "hand/wrist.h"
#include "lever/lever.h"
#include "cvra_arm_motors.h"

#define ARMS_FREQUENCY 50

extern scara_t main_arm;
extern hand_t main_hand;
extern wrist_t main_wrist;

/* Initialize arms */
void arms_init(void);

/* Auto index a motor given its name */
float arms_motor_auto_index(const char* motor_name, int motor_dir, float motor_speed);

/* Run arms controller thread */
void arms_controller_start(void);

/* Index a whole bunch of motors simultaneously */
void arms_auto_index(cvra_arm_motor_t** motors, float* motor_speeds, size_t num_motors);

#ifdef __cplusplus
}
#endif
#endif
