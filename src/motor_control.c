#include "motor_control.h"

float m1_vel_setpt;
float m2_vel_setpt;

trajectory_t robot_trajectory;
mutex_t robot_trajectory_lock;

