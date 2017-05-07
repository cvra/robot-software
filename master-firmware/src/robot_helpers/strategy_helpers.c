#include <error/error.h>
#include "position_manager/position_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

#include "math_helpers.h"
#include "trajectory_helpers.h"
#include "beacon_helpers.h"

#include "scara/scara_trajectories.h"
#include "scara/scara.h"

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

unsigned strategy_set_arm_trajectory(scara_t* arm, enum strat_color_t color, arm_waypoint_t* trajectory, unsigned trajectory_length)
{
    unsigned duration = 0;

    scara_trajectory_init(&arm->trajectory);

    for (size_t i = 0; i < trajectory_length; i++) {
        scara_trajectory_append_point_with_length(
            &arm->trajectory,
            MIRROR_X(color, trajectory[i].x),
            trajectory[i].y,
            trajectory[i].z,
            RADIANS(MIRROR_A(color, trajectory[i].a)),
            trajectory[i].coord,
            (float)trajectory[i].dt * 0.001,
            arm->length[0],
            arm->length[1],
            trajectory[i].l3);
        duration += trajectory[i].dt;
    }

    scara_do_trajectory(arm, &arm->trajectory);

    return duration;
}
