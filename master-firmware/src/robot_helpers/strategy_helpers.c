#include "position_manager/position_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "trajectory_helpers.h"

#include "strategy_helpers.h"

void strategy_map_setup(int32_t robot_size)
{
    /* Define table borders */
    polygon_set_boundingbox(robot_size/2, robot_size/2,
                            3000 - robot_size/2, 2000 - robot_size/2);

    /* Add obstacles */
    poly_t *obstacle = oa_new_poly(4);
    oa_poly_set_point(obstacle, 500, 400, 0);
    oa_poly_set_point(obstacle, 500, 700, 1);
    oa_poly_set_point(obstacle, 800, 700, 2);
    oa_poly_set_point(obstacle, 800, 400, 3);
}

void strategy_auto_position(
        int32_t x, int32_t y, int32_t heading, int32_t robot_size,
        enum strat_color_t robot_color,
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct robot_position* robot_pos,
        struct blocking_detection* robot_distance_blocking,
        struct blocking_detection* robot_angle_blocking)
{
    /* Configure robot to be slower and less sensitive to collisions */
    trajectory_set_mode_aligning(robot_mode, robot_traj, robot_distance_blocking, robot_angle_blocking);

    /* Go backwards until we hit the wall and reset position */
    trajectory_align_with_wall(robot_mode, robot_traj, robot_distance_blocking, robot_angle_blocking);
    position_set(robot_pos, MIRROR_X(robot_color, robot_size/2), 0, 0);

    /* On se mets a la bonne position en x. */
    trajectory_d_rel(robot_traj, (double)(x - robot_size/2));
    trajectory_wait_for_finish(robot_traj);

    /* On se tourne face a la paroi en Y. */
    trajectory_only_a_abs(robot_traj, 90);
    trajectory_wait_for_finish(robot_traj);

    /* On recule jusqu'a avoir touche le bord. */
    trajectory_align_with_wall(robot_mode, robot_traj, robot_distance_blocking, robot_angle_blocking);

    /* On reregle la position. */
    robot_pos->pos_d.y = robot_size / 2;
    robot_pos->pos_s16.y = robot_size / 2;

    /* On se met en place a la position demandee. */
    trajectory_set_speed(robot_traj, speed_mm2imp(robot_traj, 300), speed_rd2imp(robot_traj, 2.5));

    trajectory_d_rel(robot_traj, (double)(y - robot_size/2));
    trajectory_wait_for_finish(robot_traj);

    /* Pour finir on s'occuppe de l'angle. */
    trajectory_a_abs(robot_traj, (double)heading);
    trajectory_wait_for_finish(robot_traj);

    /* Restore robot to game mode: faster and more sensitive to collision */
    trajectory_set_mode_game(robot_mode, robot_traj, robot_distance_blocking, robot_angle_blocking);
}
