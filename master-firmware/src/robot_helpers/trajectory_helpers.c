#include "trajectory_manager/trajectory_manager.h"
#include "base/base_controller.h"

#include "robot_helpers/trajectory_helpers.h"

void trajectory_wait_for_finish(void)
{
    while(!trajectory_finished(&robot.traj)) {
        chThdSleepMilliseconds(1);
    }
}

void trajectory_wait_for_collision(void)
{
    while(!bd_get(&robot.distance_bd)) {
        chThdSleepMilliseconds(1);
    }
}
