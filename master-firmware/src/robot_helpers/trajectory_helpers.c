#include <ch.h>

#include "robot_helpers/trajectory_helpers.h"

void trajectory_wait_for_finish(struct trajectory* robot_traj)
{
    while(!trajectory_finished(robot_traj)) {
        chThdSleepMilliseconds(1);
    }
}

void trajectory_wait_for_collision(struct blocking_detection* distance_blocking)
{
    while(!bd_get(distance_blocking)) {
        chThdSleepMilliseconds(1);
    }
}
