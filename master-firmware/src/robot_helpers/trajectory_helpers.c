#include <ch.h>
#include "base/base_controller.h"

#include "trajectory_helpers.h"

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

void trajectory_align_with_wall(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking)
{
    /* Disable angle control */
    *robot_mode = BOARD_MODE_DISTANCE_ONLY;

    /* Move backwards until we hit a wall */
    trajectory_d_rel(robot_traj, -2000.);
    trajectory_wait_for_collision(distance_blocking);

    /* Stop moving on collision */
    trajectory_hardstop(robot_traj);
    bd_reset(distance_blocking);
    bd_reset(angle_blocking);

    /* Enable angle control back */
    *robot_mode = BOARD_MODE_ANGLE_DISTANCE;
}

void trajectory_move_to(struct trajectory* robot_traj, int32_t x_mm, int32_t y_mm, int32_t a_deg)
{
    trajectory_goto_xy_abs(robot_traj, x_mm, y_mm);
    trajectory_wait_for_finish(robot_traj);

    trajectory_a_abs(robot_traj, a_deg);
    trajectory_wait_for_finish(robot_traj);
}
