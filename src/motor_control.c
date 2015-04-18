#include "motor_control.h"

float m1_vel_setpt;
float m2_vel_setpt;

trajectory_frame_t demo_traj[DEMO_TRAJ_LEN];
mutex_t demo_traj_lock;

