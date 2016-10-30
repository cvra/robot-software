#include <ch.h>

#include "trajectory_manager/trajectory_manager_utils.h"
#include "trajectory_helpers.h"


void trajectory_wait_for_finish(struct trajectory* robot_traj)
{
#ifndef TESTS
    chThdSleepMilliseconds(100);
    while(!trajectory_finished(robot_traj)) {
        chThdSleepMilliseconds(1);
    }
#else
    (void)robot_traj;
#endif
}

void trajectory_wait_for_collision(struct blocking_detection* distance_blocking)
{
#ifndef TESTS
    chThdSleepMilliseconds(100);
    while(!bd_get(distance_blocking)) {
        chThdSleepMilliseconds(1);
    }
#else
    (void)distance_blocking;
#endif
}

int trajectory_has_ended(struct _robot *robot, int end_reason)
{
    if ((end_reason & TRAJ_END_GOAL_REACHED) && trajectory_finished(&robot->traj)) {
        return TRAJ_END_GOAL_REACHED;
    }

    return 0;
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

void trajectory_set_mode_aligning(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking)
{
    (void)angle_blocking;

    /* Disable angular control */
    *robot_mode = BOARD_MODE_DISTANCE_ONLY;

    /* Decrease sensitivity to collision */
    bd_set_thresholds(distance_blocking, 20000, 2);

    /* Slow down motion speed/acceleration */
    trajectory_set_speed(robot_traj,
            speed_mm2imp(robot_traj, 100.),
            speed_rd2imp(robot_traj, 0.75));
    trajectory_set_acc(robot_traj,
            acc_mm2imp(robot_traj, 150.),
            acc_rd2imp(robot_traj, 1.57));
}

void trajectory_set_mode_game(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking)
{
    /* Enable simulaneous distance and angular control */
    *robot_mode = BOARD_MODE_ANGLE_DISTANCE;

    /* Increase sensitivity to collision */
    bd_set_thresholds(angle_blocking, 12500, 1);
    bd_set_thresholds(distance_blocking, 15000, 1);

    /* Speed up motion speed/acceleration */
    trajectory_set_speed(robot_traj,
            speed_mm2imp(robot_traj, 200.),
            speed_rd2imp(robot_traj, 3.));
    trajectory_set_acc(robot_traj,
            acc_mm2imp(robot_traj, 300.),
            acc_rd2imp(robot_traj, 3.));
}
