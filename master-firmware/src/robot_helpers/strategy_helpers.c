#include <error/error.h>
#include "position_manager/position_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

#include "math_helpers.h"
#include "trajectory_helpers.h"
#include "beacon_helpers.h"

#include "strategy_helpers.h"


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
                     (double)(-robot.calibration_direction *
                              (MIRROR_X(robot_color, x) - robot.alignement_length)));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Turn to face the wall in Y */
    trajectory_only_a_abs(&robot.traj, MIRROR_A(robot_color, -90));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    /* Go forward until we hit the wall and reset position */
    trajectory_align_with_wall();

    /* Reset position in y */
    robot.pos.pos_d.y = robot.alignement_length;
    robot.pos.pos_s16.y = robot.alignement_length;

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
    robot.pos.pos_d.y = robot.alignement_length;
    robot.pos.pos_s16.y = robot.alignement_length;

    trajectory_set_speed(&robot.traj, speed_mm2imp(&robot.traj, 300),
                         speed_rd2imp(&robot.traj, 2.5));

    trajectory_d_rel(&robot.traj,
                     (double)(-robot.calibration_direction * (y - robot.alignement_length)));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    trajectory_set_mode_game(&robot.mode, &robot.traj, &robot.distance_bd, &robot.angle_bd);
}

static point_t point(float x, float y)
{
    point_t res = {.x = x, .y = y};
    return res;
}

static point_t block_pos_in_blocks_frame(enum block_color color)
{
    const float block_size = 60.f; // in mm

    switch(color) {
        case BLOCK_GREEN:   return point(  block_size,            0);
        case BLOCK_BLUE:    return point(           0,   block_size);
        case BLOCK_RED:     return point(- block_size,            0);
        case BLOCK_BLACK:   return point(           0, - block_size);
        case BLOCK_YELLOW:
        default:            return point(           0,            0);
    }
}

point_t strategy_block_pos(se2_t blocks_pose, enum block_color color)
{
    return se2_transform(blocks_pose, block_pos_in_blocks_frame(color));
}
