#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ch.h>
#include <odometry/robot_base.h>
#include "trajectories.h"

#define ROBOT_TRAJ_LEN 100

#ifdef __cplusplus
extern "C" {
#endif


extern float m1_vel_setpt;
extern float m2_vel_setpt;

extern trajectory_t robot_trajectory;
extern mutex_t robot_trajectory_lock;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
