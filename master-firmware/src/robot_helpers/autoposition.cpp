#include <aversive/blocking_detection_manager/blocking_detection_manager.h>
#include <aversive/position_manager/position_manager.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>

#include "trajectory_helpers.h"

#include "autoposition.h"

void strategy_auto_position(int32_t x, int32_t y, int32_t heading, enum strat_color_t robot_color)
{
    /* Configure  to be slower and less sensitive to collisions */
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);

    /* Go forward until we hit the wall and reset position */
    trajectory_align_with_wall();

    /* Set  position in x and heading */
    if (robot.calibration_direction < 0) {
        position_set(&robot.pos, MIRROR_X(robot_color, robot.alignement_length), 0,
                     MIRROR_A(robot_color, 0));
    } else {
        position_set(&robot.pos, MIRROR_X(robot_color, robot.alignement_length), 0,
                     MIRROR_A(robot_color, 180));
    }

    /* Go to desired position in x. */
    trajectory_d_rel(&robot.traj,
                     (double)(-robot.calibration_direction * (MIRROR_X(robot_color, x) - robot.alignement_length)));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Turn to face the wall in Y */
    trajectory_only_a_abs(&robot.traj, MIRROR_A(robot_color, -90));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Go forward until we hit the wall and reset position */
    trajectory_align_with_wall();

    /* Reset position in y */
    {
        absl::MutexLock l(&robot.pos.lock_);
        robot.pos.pos_d.y = robot.alignement_length;
        robot.pos.pos_s16.y = robot.alignement_length;
    }

    /* Go to the desired position in y */
    trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 300),
                         speed_rd2imp(&robot.traj, 2.5));

    trajectory_d_rel(&robot.traj,
                     (double)(-robot.calibration_direction * (y - robot.alignement_length)));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Set the angle to the desired value */
    trajectory_a_abs(&robot.traj, (double)heading);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Restore  to game mode: faster and more sensitive to collision */
    trajectory_set_mode_game(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);
}

void strategy_align_y(int32_t y)
{
    trajectory_set_mode_aligning(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);

    trajectory_align_with_wall();
    {
        absl::MutexLock l(&robot.pos.lock_);
        robot.pos.pos_d.y = robot.alignement_length;
        robot.pos.pos_s16.y = robot.alignement_length;
    }

    trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 300),
                         speed_rd2imp(&robot.traj, 2.5));

    trajectory_d_rel(&robot.traj,
                     (double)(-robot.calibration_direction * (y - robot.alignement_length)));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    trajectory_set_mode_game(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);
}
