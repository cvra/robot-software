#include "motor_control.h"

float m1_vel_setpt;
float m2_vel_setpt;

struct robot_base_pose_2d_s robot_pose;
mutex_t robot_pose_lock;
