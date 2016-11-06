#include <ch.h>

#include "trajectory_manager/trajectory_manager_utils.h"
#include "trajectory_helpers.h"


int trajectory_wait_for_end(struct _robot *robot, int end_reason)
{
#ifndef TESTS
    chThdSleepMilliseconds(100);
#endif

    int traj_end_reason = 0;
    while(traj_end_reason == 0) {
        traj_end_reason = trajectory_has_ended(robot, end_reason);
#ifndef TESTS
        chThdSleepMilliseconds(1);
#endif
    }

    return traj_end_reason;
}

int trajectory_has_ended(struct _robot *robot, int end_reason)
{
    if ((end_reason & TRAJ_END_GOAL_REACHED) && trajectory_finished(&robot->traj)) {
        return TRAJ_END_GOAL_REACHED;
    }

    if ((end_reason & TRAJ_END_COLLISION) && bd_get(&robot->distance_bd)) {
        return TRAJ_END_COLLISION;
    }

    if ((end_reason & TRAJ_END_COLLISION) && bd_get(&robot->angle_bd)) {
        return TRAJ_END_COLLISION;
    }

    return 0;
}

void trajectory_align_with_wall(struct _robot *robot)
{
    /* Disable angle control */
    robot->mode = BOARD_MODE_DISTANCE_ONLY;

    /* Move backwards until we hit a wall */
    trajectory_d_rel(&robot->traj, -2000.);
    trajectory_wait_for_end(robot, TRAJ_END_COLLISION);

    /* Stop moving on collision */
    trajectory_hardstop(&robot->traj);
    bd_reset(&robot->distance_bd);
    bd_reset(&robot->angle_bd);

    /* Enable angle control back */
    robot->mode = BOARD_MODE_ANGLE_DISTANCE;
}

void trajectory_move_to(struct _robot* robot, int32_t x_mm, int32_t y_mm, int32_t a_deg)
{
    trajectory_goto_xy_abs(&robot->traj, x_mm, y_mm);
    trajectory_wait_for_end(robot, TRAJ_END_GOAL_REACHED);

    trajectory_a_abs(&robot->traj, a_deg);
    trajectory_wait_for_end(robot, TRAJ_END_GOAL_REACHED);
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
