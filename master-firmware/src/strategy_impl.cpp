#include <error/error.h>
#include <aversive/blocking_detection_manager/blocking_detection_manager.h>

#include <aversive/position_manager/position_manager.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/motor_helpers.h"
#include "robot_helpers/arm_helpers.h"

#include "control_panel.h"
#include "base/map_server.h"
#include "protobuf/sensors.pb.h"
#include "config.h"
#include "main.h"

#include "strategy_impl.h"

void strategy_stop_robot(strategy_context_t* strat)
{
    trajectory_stop(&strat->robot->traj);
    strat->wait_ms(200);
    strat->robot->mode = BOARD_MODE_FREE;
    strat->wait_ms(200);
    strat->robot->mode = BOARD_MODE_ANGLE_DISTANCE;
}

bool strategy_goto_avoid(strategy_context_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags)
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

bool strategy_goto_avoid_retry(strategy_context_t* strat, int x_mm, int y_mm, int a_deg, int traj_end_flags, int num_retries)
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

void strategy_align_front_sensors(strategy_context_t* strat)
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

bool IndexArms::execute(RobotState& state)
{
    NOTICE("Indexing arms!");

    // set index when user presses color button, so indexing is done manually
    float offsets[3];

    offsets[0] = motor_get_position("theta-1");
    offsets[1] = motor_get_position("theta-2");
    offsets[2] = motor_get_position("theta-3");
    float right_directions[3] = {-1, -1, 1};
    arm_compute_offsets(RIGHT_ARM_REFS, right_directions, offsets);

    parameter_scalar_set(PARAMETER("master/arms/right/offsets/q1"), offsets[0]);
    parameter_scalar_set(PARAMETER("master/arms/right/offsets/q2"), offsets[1]);
    parameter_scalar_set(PARAMETER("master/arms/right/offsets/q3"), offsets[2]);

    strat->wait_ms(500);
    strat->wait_for_user_input();

    offsets[0] = motor_get_position("left-theta-1");
    offsets[1] = motor_get_position("left-theta-2");
    offsets[2] = motor_get_position("left-theta-3");
    float left_directions[3] = {1, 1, -1};
    arm_compute_offsets(LEFT_ARM_REFS, left_directions, offsets);

    parameter_scalar_set(PARAMETER("master/arms/left/offsets/q1"), offsets[0]);
    parameter_scalar_set(PARAMETER("master/arms/left/offsets/q2"), offsets[1]);
    parameter_scalar_set(PARAMETER("master/arms/left/offsets/q3"), offsets[2]);

    strat->wait_ms(500);
    strat->wait_for_user_input();

    state.arms_are_indexed = true;
    return true;
}

bool RetractArms::execute(RobotState& state)
{
    NOTICE("Retracting arms!");

    strat->gripper_set(RIGHT, GRIPPER_OFF);
    strat->manipulator_goto(RIGHT, MANIPULATOR_RETRACT);

    state.has_puck = false;
    state.arms_are_deployed = false;
    return true;
}

bool TakePuck::execute(RobotState& state)
{
    float x, y, a;
    if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
        x = MIRROR_X(strat->color, pucks[puck_id].pos_x_mm - 170);
        y = pucks[puck_id].pos_y_mm + MIRROR(strat->color, 50);
        a = MIRROR_A(strat->color, 180);
    } else {
        x = MIRROR_X(strat->color, pucks[puck_id].pos_x_mm) - 50;
        y = pucks[puck_id].pos_y_mm - 260;
        a = MIRROR_A(strat->color, -90);
    }

    if (!strategy_goto_avoid(strat, x, y, a, TRAJ_FLAGS_ALL)) {
        return false;
    }

    if (pucks[puck_id].orientation == PuckOrientiation_VERTICAL) {
        strategy_align_front_sensors(strat);
    }

    state.arms_are_deployed = true;
    strat->gripper_set(RIGHT, GRIPPER_ACQUIRE);

    if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
        strat->manipulator_goto(RIGHT, MANIPULATOR_PICK_HORZ);
    } else {
        strat->manipulator_goto(RIGHT, MANIPULATOR_PICK_VERT);
    }
    strat->wait_ms(500);
    strat->manipulator_goto(RIGHT, MANIPULATOR_LIFT_HORZ);

    state.puck_available[puck_id] = false;

    if (!strat->puck_is_picked()) {
        strat->gripper_set(RIGHT, GRIPPER_OFF);
        return false;
    }

    state.has_puck = true;
    state.has_puck_color = pucks[puck_id].color;
    return true;
}

bool DepositPuck::execute(RobotState& state)
{
    float x = MIRROR_X(strat->color, areas[zone_id].pos_x_mm);
    float y = areas[zone_id].pos_y_mm - MIRROR(strat->color, 50);
    float a = MIRROR_A(strat->color, 0);

    if (!strategy_goto_avoid(strat, x, y, a, TRAJ_FLAGS_ALL)) {
        return false;
    }
    strat->gripper_set(RIGHT, GRIPPER_RELEASE);
    strat->wait_ms(100);

    strat->gripper_set(RIGHT, GRIPPER_OFF);

    pucks_in_area++;
    state.has_puck = false;
    state.classified_pucks[areas[zone_id].color]++;
    state.arms_are_deployed = true;
    return true;
}

bool LaunchAccelerator::execute(RobotState& state)
{
    float x = (strat->color == VIOLET) ? 1695 : 1405;

    if (!strategy_goto_avoid(strat, x, 330, MIRROR_A(strat->color, 90), TRAJ_FLAGS_ALL)) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->manipulator_goto(RIGHT, MANIPULATOR_DEPLOY_FULLY);
    trajectory_d_rel(&strat->robot->traj, -30);

    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    trajectory_a_rel(&strat->robot->traj, MIRROR(strat->color, 20));
    trajectory_wait_for_end(TRAJ_FLAGS_ROTATION);
    trajectory_d_rel(&strat->robot->traj, 40);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    state.accelerator_is_done = true;
    return true;
}

bool TakeGoldonium::execute(RobotState& state)
{
    float x = (strat->color == VIOLET) ? 2275 : 825;

    if (!strategy_goto_avoid(strat, x, 400, MIRROR_A(strat->color, 90), TRAJ_FLAGS_ALL)) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->manipulator_goto(RIGHT, MANIPULATOR_PICK_GOLDONIUM);

    if (!strategy_goto_avoid(strat, x, 330, MIRROR_A(strat->color, 90), TRAJ_FLAGS_ALL)) {
        return false;
    }

    strat->gripper_set(RIGHT, GRIPPER_ACQUIRE);
    trajectory_d_rel(&strat->robot->traj, -27);
    strat->wait_ms(1500);

    if (!strat->puck_is_picked()) {
        strat->gripper_set(RIGHT, GRIPPER_OFF);
        trajectory_d_rel(&strat->robot->traj, 80);
        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
        return false;
    }

    strat->manipulator_goto(RIGHT, MANIPULATOR_LIFT_GOLDONIUM);
    strat->wait_ms(500);
    strat->gripper_set(RIGHT, GRIPPER_OFF);

    trajectory_d_rel(&strat->robot->traj, 80);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    state.goldonium_in_house = false;
    state.has_goldonium = true;
    return true;
}
