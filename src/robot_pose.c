#include <ch.h>
#include "robot_pose.h"

struct robot_base_pose_2d_s robot_pose;
mutex_t robot_pose_lock;
