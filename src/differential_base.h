#ifndef DIFFERENTIAL_BASE_H
#define DIFFERENTIAL_BASE_H

#include <ch.h>
#include <odometry/robot_base.h>
#include "trajectories.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DIFF_BASE_TRAJ_LENGTH 100

extern trajectory_t diff_base_trajectory;
extern mutex_t diff_base_trajectory_lock;


void differential_base_init(void);
void differential_base_tracking_start(void);

#ifdef __cplusplus
}
#endif

#endif /* DIFFERENTIAL_BASE_H */
