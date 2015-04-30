#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ch.h>
#include <odometry/robot_base.h>

#define ROBOT_TRAJ_LEN 100

#ifdef __cplusplus
extern "C" {
#endif

extern float m1_vel_setpt;
extern float m2_vel_setpt;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
