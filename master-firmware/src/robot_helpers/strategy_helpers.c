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
                     (double)(-robot.calibration_direction * (MIRROR_X(robot_color, x) - robot.alignement_length)));
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

static point_t cube_pos_in_block_frame(enum cube_color color, enum strat_color_t robot_color)
{
    const float cube_size = 60.f; // in mm

    switch (color) {
        case CUBE_GREEN:
            return point(MIRROR(robot_color, -cube_size), 0);
        case CUBE_BLUE:
            return point(MIRROR(robot_color, 0), +cube_size);
        case CUBE_ORANGE:
            return point(MIRROR(robot_color, +cube_size), 0);
        case CUBE_BLACK:
            return point(MIRROR(robot_color, 0), -cube_size);
        case CUBE_YELLOW:
        default:
            return point(MIRROR(robot_color, 0), 0);
    }
}

point_t strategy_cube_pos(se2_t cubes_pose, enum cube_color color, enum strat_color_t robot_color)
{
    return se2_transform(cubes_pose, cube_pos_in_block_frame(color, robot_color));
}

float strategy_flight_distance_to_goal(point_t pos, point_t goal)
{
    return pt_norm(&pos, &goal);
}

float strategy_distance_to_goal(point_t pos, point_t goal)
{
    oa_start_end_points(pos.x, pos.y, goal.x, goal.y);
    oa_process();

    point_t* points;
    int num_points = oa_get_path(&points);

    if (num_points <= 0) {
        return INFINITY;
    }

    float distance = pt_norm(&pos, &points[0]);
    for (int i = 1; i < num_points; i++) {
        distance += pt_norm(&points[i - 1], &points[i]);
    }

    return distance;
}

void strategy_sort_poses_by_distance(se2_t current_pose, se2_t* pickup_poses, int num_poses, float (*distance_metric)(point_t, point_t))
{
    /* Compute distances to current position */
    const point_t current_pos = {current_pose.translation.x, current_pose.translation.y};
    float distances[num_poses];
    for (int i = 0; i < num_poses; i++) {
        const point_t candidate = {pickup_poses[i].translation.x, pickup_poses[i].translation.y};
        distances[i] = (*distance_metric)(current_pos, candidate);
    }

    /* Sort by distance using selection sort */
    for (int i = 0; i < num_poses - 1; i++) {
        int min_index = argmin(distances + i, num_poses - i) + i;

        /* Swap distances */
        float min_distance = distances[min_index];
        distances[min_index] = distances[i];
        distances[i] = min_distance;

        /* Swap poses */
        se2_t min_pose = pickup_poses[min_index];
        pickup_poses[min_index] = pickup_poses[i];
        pickup_poses[i] = min_pose;
    }
}

shoulder_mode_t MIRROR_SHOULDER(enum strat_color_t color, shoulder_mode_t mode)
{
    if (color == YELLOW) {
        return mode;
    } else {
        return mode == SHOULDER_BACK ? SHOULDER_FRONT : SHOULDER_BACK;
    }
}

enum lever_side_t MIRROR_LEVER(enum strat_color_t color, enum lever_side_t side)
{
    if (color == YELLOW) {
        return side;
    } else {
        return side == LEVER_SIDE_LEFT ? LEVER_SIDE_RIGHT : LEVER_SIDE_LEFT;
    }
}
