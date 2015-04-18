#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ch.h>
#include "trajectories.h"

#define DEMO_TRAJ_LEN 10

#ifdef __cplusplus
extern "C" {
#endif


extern float m1_vel_setpt;
extern float m2_vel_setpt;

extern trajectory_frame_t demo_traj[];
extern mutex_t demo_traj_lock;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
