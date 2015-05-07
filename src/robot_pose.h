#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include "odometry/robot_base.h"

extern struct robot_base_pose_2d_s robot_pose;
extern mutex_t robot_pose_lock;

#endif
