#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ch.h>
#include "trajectories.h"

#define ROBOT_TRAJ_LEN 100

#ifdef __cplusplus
extern "C" {
#endif


extern float m1_vel_setpt;
extern float m2_vel_setpt;

struct robot_traj_s {
    /** x position */
    trajectory_frame_t x[ROBOT_TRAJ_LEN];
    /** y position */
    trajectory_frame_t y[ROBOT_TRAJ_LEN];

    /** Heading */
    trajectory_frame_t theta[ROBOT_TRAJ_LEN];

    /** Linear speed */
    trajectory_frame_t speed[ROBOT_TRAJ_LEN];
    /** Angular speed */
    trajectory_frame_t omega[ROBOT_TRAJ_LEN];

    /** Guarantees exclusive access to the structure. */
    mutex_t lock;
};

extern struct robot_traj_s robot_traj;


#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
