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

static point_t cube_pos_in_block_frame(enum cube_color color)
{
    const float cube_size = 60.f; // in mm

    switch(color) {
        case CUBE_GREEN:    return point(  cube_size,            0);
        case CUBE_BLUE:     return point(           0,   cube_size);
        case CUBE_RED:      return point(- cube_size,            0);
        case CUBE_BLACK:    return point(           0, - cube_size);
        case CUBE_YELLOW:
        default:            return point(           0,           0);
    }
}

point_t strategy_cube_pos(se2_t cubes_pose, enum cube_color color)
{
    return se2_transform(cubes_pose, cube_pos_in_block_frame(color));
}

se2_t strategy_closest_pose_to_pickup_cubes(se2_t current_pose, se2_t cubes_pose)
{
    se2_t closest_pose;

    point_t current_pos = {current_pose.translation.x, current_pose.translation.y};
    point_t candidates[4] = {
        {cubes_pose.translation.x - 160, cubes_pose.translation.y - 160},
        {cubes_pose.translation.x + 160, cubes_pose.translation.y - 160},
        {cubes_pose.translation.x + 160, cubes_pose.translation.y + 160},
        {cubes_pose.translation.x - 160, cubes_pose.translation.y + 160}
    };

    int argmin = 0;
    float min_dist = 1e10;
    for (int i = 0; i < 4; i++) {
        float dist = pt_norm(&current_pos, &candidates[i]);
        if (dist < min_dist) {
            argmin = i;
            min_dist = dist;
        }
    }

    closest_pose.translation.x = candidates[argmin].x;
    closest_pose.translation.y = candidates[argmin].y;
    closest_pose.rotation.angle = -45 + 90 * argmin;

    return closest_pose;
}
