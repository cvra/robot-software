#include <ch.h>

#include <timestamp/timestamp.h>
#include <error/error.h>

#include "trajectory_manager/trajectory_manager_utils.h"
#include "trajectory_manager/trajectory_manager_core.h"
#include "base/map.h"
#include "math_helpers.h"
#include "beacon_helpers.h"
#include "protobuf/beacons.pb.h"

#include "trajectory_helpers.h"

// TODO Fix this, this should live in a header somewhere probably
extern messagebus_t bus;


int trajectory_wait_for_end(int watched_end_reasons)
{
#ifndef TESTS
    chThdSleepMilliseconds(100);
#endif

    int traj_end_reason = 0;
    while (traj_end_reason == 0) {
        traj_end_reason = trajectory_has_ended(watched_end_reasons);
#ifndef TESTS
        chThdSleepMilliseconds(1);
#endif
    }
    NOTICE("End of trajectory reason %d at %d %d %d",
           traj_end_reason, position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos),
           position_get_a_deg_s16(&robot.pos));

    return traj_end_reason;
}

int trajectory_has_ended(int watched_end_reasons)
{
    if ((watched_end_reasons & TRAJ_END_GOAL_REACHED) && trajectory_finished(&robot.traj)) {
        return TRAJ_END_GOAL_REACHED;
    }

    if ((watched_end_reasons & TRAJ_END_NEAR_GOAL) && trajectory_nearly_finished(&robot.traj)) {
        return TRAJ_END_NEAR_GOAL;
    }

    if ((watched_end_reasons & TRAJ_END_COLLISION) && bd_get(&robot.distance_bd)) {
        return TRAJ_END_COLLISION;
    }

    if ((watched_end_reasons & TRAJ_END_COLLISION) && bd_get(&robot.angle_bd)) {
        return TRAJ_END_COLLISION;
    }

    if (watched_end_reasons & TRAJ_END_OPPONENT_NEAR) {
        BeaconSignal beacon_signal;
        messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");
        messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

        // only consider recent beacon signal
        if (timestamp_duration_s(beacon_signal.timestamp.us,
                                 timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
            if (beacon_signal.range.range.distance < TRAJ_MIN_DISTANCE_TO_OPPONENT) {
                float x_opp, y_opp;
                beacon_cartesian_convert(&robot.pos,
                                         1000 * beacon_signal.range.range.distance,
                                         beacon_signal.range.angle,
                                         &x_opp,
                                         &y_opp);

                if (trajectory_is_on_collision_path(&robot, x_opp, y_opp)) {
                    return TRAJ_END_OPPONENT_NEAR;
                }
            }
        }
    }

    if (watched_end_reasons & TRAJ_END_ALLY_NEAR) {
        messagebus_topic_t *topic = messagebus_find_topic(&bus, "/allied_position");
        AlliedPosition pos;

        if (topic && messagebus_topic_read(topic, &pos, sizeof(pos))) {
            uint32_t age = timestamp_duration_s(pos.timestamp.us, timestamp_get());

            if (age < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
                if (trajectory_is_on_collision_path(&robot, pos.x, pos.y)) {
                    return TRAJ_END_ALLY_NEAR;
                }
            }
        }
    }

    if (watched_end_reasons & TRAJ_END_TIMER && trajectory_get_time() >= GAME_DURATION) {
        trajectory_hardstop(&robot.traj);
        return TRAJ_END_TIMER;
    }

    return 0;
}

void trajectory_align_with_wall(void)
{
    /* Disable angle control */
    robot.mode = BOARD_MODE_DISTANCE_ONLY;

    /* Move in direction until we hit a wall */
    trajectory_d_rel(&robot.traj, robot.calibration_direction * 2000.);
    trajectory_wait_for_end(TRAJ_END_COLLISION);

    /* Stop moving on collision */
    trajectory_hardstop(&robot.traj);
    bd_reset(&robot.distance_bd);
    bd_reset(&robot.angle_bd);

    /* Enable angle control back */
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
}

void trajectory_move_to(int32_t x_mm, int32_t y_mm, int32_t a_deg)
{
    trajectory_goto_xy_abs(&robot.traj, x_mm, y_mm);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    trajectory_a_abs(&robot.traj, a_deg);
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
}

static bool trajectory_is_cartesian(struct trajectory* traj)
{
    return traj->state == RUNNING_XY_START ||
           traj->state == RUNNING_XY_ANGLE ||
           traj->state == RUNNING_XY_ANGLE_OK ||
           traj->state == RUNNING_XY_F_START ||
           traj->state == RUNNING_XY_F_ANGLE ||
           traj->state == RUNNING_XY_F_ANGLE_OK ||
           traj->state == RUNNING_XY_B_START ||
           traj->state == RUNNING_XY_B_ANGLE ||
           traj->state == RUNNING_XY_B_ANGLE_OK;
}

bool trajectory_crosses_obstacle(struct _robot* robot, poly_t* opponent, point_t* intersection)
{
    point_t current_position = {
            position_get_x_float(&robot->pos),
            position_get_y_float(&robot->pos)
        };
    point_t target_position;

    if (trajectory_is_cartesian(&robot->traj)) {
        target_position.x = robot->traj.target.cart.x;
        target_position.y = robot->traj.target.cart.y;
    } else {
        vect2_pol delta_pol;
        delta_pol.r = robot->traj.target.pol.distance - (float)rs_get_distance(&robot->rs);
        delta_pol.theta = robot->traj.target.pol.angle - (float)rs_get_angle(&robot->rs);

        // Account for current heading
        delta_pol.theta += position_get_a_deg_s16(&robot->pos);

        vect2_cart delta_xy;
        vect2_pol2cart(&delta_pol, &delta_xy);

        target_position.x = current_position.x + delta_xy.x;
        target_position.y = current_position.y + delta_xy.y;
    }

    uint8_t path_crosses_obstacle = is_crossing_poly(current_position, target_position, intersection, opponent);
    bool current_pos_inside_obstacle =
        math_point_is_in_square(opponent, position_get_x_s16(&robot->pos), position_get_y_s16(&robot->pos));

    return path_crosses_obstacle == 1 || current_pos_inside_obstacle;
}

bool trajectory_is_on_collision_path(struct _robot* robot, int x, int y)
{
    point_t points[4];
    poly_t opponent = {.pts = points, .l = 4};
    map_set_rectangular_obstacle(&opponent,
                                 x,
                                 y,
                                 robot->opponent_size,
                                 robot->opponent_size,
                                 robot->robot_size);

    point_t intersection;
    return trajectory_crosses_obstacle(robot, &opponent, &intersection);
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
                         speed_mm2imp(robot_traj, 300.),
                         speed_rd2imp(robot_traj, 3.));
    trajectory_set_acc(robot_traj,
                       acc_mm2imp(robot_traj, 600.),
                       acc_rd2imp(robot_traj, 6.));
}

void trajectory_game_timer_reset(void)
{
    robot.start_time = timestamp_get();
}

int trajectory_get_time(void)
{
    return timestamp_duration_s(robot.start_time, timestamp_get());
}

int trajectory_get_time_ms(void)
{
    return timestamp_duration_us(robot.start_time, timestamp_get()) / 1000;
}

bool trajectory_game_has_ended(void)
{
    return trajectory_get_time() >= GAME_DURATION;
}
