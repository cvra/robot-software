#include <error/error.h>
#include "position_manager/position_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

#include "math_helpers.h"
#include "trajectory_helpers.h"
#include "beacon_helpers.h"

#include "strategy_helpers.h"


void strategy_auto_position(
        int32_t x, int32_t y, int32_t heading, int32_t robot_size,
        enum strat_color_t robot_color, struct _robot* robot, messagebus_t* bus)
{
    /* Configure robot to be slower and less sensitive to collisions */
    trajectory_set_mode_aligning(&robot->mode, &robot->traj, &robot->distance_bd, &robot->angle_bd);

    /* Go backwards until we hit the wall and reset position */
    trajectory_align_with_wall(robot, bus);

    /* Set robot position in x and heading */
    if (robot->calibration_direction < 0) {
        position_set(&robot->pos, MIRROR_X(robot_color, robot_size/2), 0, 0);
    } else {
        position_set(&robot->pos, MIRROR_X(robot_color, robot_size/2), 0, 180);
    }

    /* Go to desired position in x. */
    trajectory_d_rel(&robot->traj, (double)(- robot->calibration_direction * (x - robot_size/2)));
    trajectory_wait_for_end(robot, bus, TRAJ_END_GOAL_REACHED);

    /* Turn to face the wall in Y */
    trajectory_only_a_rel(&robot->traj, 90);
    trajectory_wait_for_end(robot, bus, TRAJ_END_GOAL_REACHED);

    /* Go backwards until we hit the wall and  reset position */
    trajectory_align_with_wall(robot, bus);

    /* Reset position in y */
    robot->pos.pos_d.y = robot_size / 2;
    robot->pos.pos_s16.y = robot_size / 2;

    /* Go to the desired position in y */
    trajectory_set_speed(&robot->traj, speed_mm2imp(&robot->traj, 300), speed_rd2imp(&robot->traj, 2.5));

    trajectory_d_rel(&robot->traj, (double)(- robot->calibration_direction * (y - robot_size/2)));
    trajectory_wait_for_end(robot, bus, TRAJ_END_GOAL_REACHED);

    /* Set the angle to the desired value */
    trajectory_a_abs(&robot->traj, (double)heading);
    trajectory_wait_for_end(robot, bus, TRAJ_END_GOAL_REACHED);

    /* Restore robot to game mode: faster and more sensitive to collision */
    trajectory_set_mode_game(&robot->mode, &robot->traj, &robot->distance_bd, &robot->angle_bd);
}

void strategy_align_y(int32_t y, int32_t robot_size, struct _robot* robot, messagebus_t* bus)
{
    trajectory_set_mode_aligning(&robot->mode, &robot->traj, &robot->distance_bd, &robot->angle_bd);

    trajectory_align_with_wall(robot, &bus);
    robot->pos.pos_d.y = robot_size / 2;
    robot->pos.pos_s16.y = robot_size / 2;

    trajectory_set_speed(&robot->traj, speed_mm2imp(&robot->traj, 300), speed_rd2imp(&robot->traj, 2.5));

    trajectory_d_rel(&robot->traj, (double)(- robot->calibration_direction * (y - robot_size/2)));
    trajectory_wait_for_end(robot, bus, TRAJ_END_GOAL_REACHED);

    trajectory_set_mode_game(&robot->mode, &robot->traj, &robot->distance_bd, &robot->angle_bd);
}
