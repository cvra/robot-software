#include <error/error.h>
#include <aversive/blocking_detection_manager/blocking_detection_manager.h>

#include <aversive/position_manager/position_manager.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "robot_helpers/strategy_helpers.h"

#include "control_panel.h"
#include "base/map_server.h"
#include "protobuf/sensors.pb.h"

#include "strategy_impl.h"
#include "main.h"

void strategy_stop_robot(strategy_impl_t* strat)
{
    trajectory_stop(&strat->robot->traj);
    strat->wait_ms(200);
    strat->robot->mode = BOARD_MODE_FREE;
    strat->wait_ms(200);
    strat->robot->mode = BOARD_MODE_ANGLE_DISTANCE;
}

bool strategy_goto_avoid(strategy_impl_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags)
{
    auto map = map_server_map_lock_and_get();

    /* Compute path */
    const point_t start = {
        position_get_x_float(&strat->robot->pos),
        position_get_y_float(&strat->robot->pos)};
    oa_start_end_points(&map->oa, start.x, start.y, x_mm, y_mm);
    oa_process(&map->oa);

    /* Retrieve path */
    point_t* points;
    int num_points = oa_get_path(&map->oa, &points);
    DEBUG("Path to (%d, %d) computed with %d points", x_mm, y_mm, num_points);
    if (num_points <= 0) {
        WARNING("No path found!");
        strategy_stop_robot(strat);
        map_server_map_release(map);
        return false;
    }

    /* Execute path, one waypoint at a time */
    int end_reason = 0;

    for (int i = 0; i < num_points; i++) {
        DEBUG("Going to x: %.1fmm y: %.1fmm", points[i].x, points[i].y);

        trajectory_goto_xy_abs(&strat->robot->traj, points[i].x, points[i].y);

        if (i == num_points - 1) /* last point */ {
            end_reason = trajectory_wait_for_end(traj_end_flags);
        } else {
            end_reason = trajectory_wait_for_end(traj_end_flags | TRAJ_END_NEAR_GOAL);
        }

        if (end_reason != TRAJ_END_GOAL_REACHED && end_reason != TRAJ_END_NEAR_GOAL) {
            break;
        }
    }

    if (end_reason == TRAJ_END_GOAL_REACHED) {
        strat->wait_ms(200);
        trajectory_a_abs(&strat->robot->traj, a_deg);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

        DEBUG("Goal reached successfully");
        map_server_map_release(map);

        return true;
    } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
        control_panel_set(LED_PC);
        strategy_stop_robot(strat);
        strat->wait_ms(100);
        strategy_stop_robot(strat);
        control_panel_clear(LED_PC);
        WARNING("Stopping robot because opponent too close");
    } else if (end_reason == TRAJ_END_COLLISION) {
        strategy_stop_robot(strat);
        WARNING("Stopping robot because collision detected");
    } else if (end_reason == TRAJ_END_TIMER) {
        strategy_stop_robot(strat);
        WARNING("Stopping robot because game has ended !");
    } else {
        WARNING("Trajectory ended with reason %d", end_reason);
    }

    map_server_map_release(map);
    return false;
}

bool strategy_goto_avoid_retry(strategy_impl_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags, int num_retries)
{
    bool finished = false;
    int counter = 0;

    while (!finished) {
        DEBUG("Try #%d", counter);
        finished = strategy_goto_avoid(strat, x_mm, y_mm, a_deg, traj_end_flags);
        counter++;

        // Exit when maximum number of retries is reached
        // Negative number of retries means infinite number of retries
        if (num_retries >= 0 && counter > num_retries) {
            break;
        }
    }

    return finished;
}

void strategy_align_front_sensors(strategy_impl_t* strat)
{
    messagebus_topic_t *left, *right;
    Range left_range, right_range;

    const float width = 0.1f;

    left = messagebus_find_topic_blocking(&bus, "/distance/front_left");
    right = messagebus_find_topic_blocking(&bus, "/distance/front_right");

    messagebus_topic_wait(left, &left_range, sizeof(Range));
    messagebus_topic_wait(right, &right_range, sizeof(Range));

    float dx = left_range.distance - right_range.distance;
    float alpha = atan2f(dx, width);

    trajectory_a_rel(&strat->robot->traj, DEGREES(-alpha));
    trajectory_wait_for_end(TRAJ_FLAGS_ROTATION);
}
