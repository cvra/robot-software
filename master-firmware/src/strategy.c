#include <ch.h>
#include <hal.h>

#include <error/error.h>
#include <timestamp/timestamp.h>
#include <blocking_detection_manager/blocking_detection_manager.h>
#include <trajectory_manager/trajectory_manager_utils.h>
#include <obstacle_avoidance/obstacle_avoidance.h>

#include "priorities.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "base/base_controller.h"
#include "base/map.h"
#include "config.h"
#include "main.h"

#include "strategy.h"

#define STRATEGY_STACKSIZE 1024

static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
void strategy_play_game(void* robot);

static void wait_for_starter(void)
{
    /* Wait for a rising edge */
    while (palReadPad(GPIOF, GPIOF_START)) {
        chThdSleepMilliseconds(10);
    }
    while (!palReadPad(GPIOF, GPIOF_START)) {
        chThdSleepMilliseconds(10);
    }
}

static void wait_for_autoposition_signal(void)
{
    wait_for_starter();
}

bool strategy_goto_avoid(struct _robot* robot, int x_mm, int y_mm, int a_deg)
{
    /* Create obstacle at opponent position */
    beacon_signal_t beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");
    messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

    // only consider recent beacon signal
    if (timestamp_duration_s(beacon_signal.timestamp, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
        float x_opp, y_opp;
        beacon_cartesian_convert(&robot->pos, 1000 * beacon_signal.distance, beacon_signal.heading, &x_opp, &y_opp);
        map_set_opponent_obstacle(0, x_opp, y_opp, robot->opponent_size * 1.25, robot->robot_size);
    }

    /* Compute path */
    oa_reset();
    const point_t start = {
            position_get_x_s16(&robot->pos),
            position_get_y_s16(&robot->pos)
        };
    oa_start_end_points(start.x, start.y, x_mm, y_mm);
    oa_process();

    /* Retrieve path */
    point_t *points;
    int num_points = oa_get_path(&points);
    DEBUG("Path to (%d, %d) computed with %d points", x_mm, y_mm, num_points);

    /* Execute path, one waypoint at a time */
    int end_reason = 0;

    for (int i = 0; i < num_points; i++) {
        DEBUG("Going to x: %.1fmm y: %.1fmm", points[i].x, points[i].y);

        trajectory_goto_forward_xy_abs(&robot->traj, points[i].x, points[i].y);
        end_reason = trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED | TRAJ_END_OPPONENT_NEAR);

        if (end_reason == TRAJ_END_OPPONENT_NEAR) {
            break;
        }
    }

    if (end_reason == TRAJ_END_GOAL_REACHED) {
        trajectory_a_abs(&robot->traj, a_deg);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

        DEBUG("Goal reached successfully");

        return true;
    } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
        trajectory_hardstop(&robot->traj);
        rs_set_distance(&robot->rs, 0);
        rs_set_angle(&robot->rs, 0);

        WARNING("Stopping robot because opponent too close");
    } else {
        WARNING("Trajectory ended with reason %d", end_reason);
    }

    return false;
}


void strategy_play_game(void* _robot)
{
    chRegSetThreadName("strategy");

    struct _robot* robot = (struct _robot*)_robot;
    enum strat_color_t color = YELLOW;

    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot\n");
    strategy_auto_position(600, 200, 90, robot->robot_size, color, robot, &bus);
    NOTICE("Robot positioned at x: 600[mm], y: 200[mm], a: 90[deg]\n");

    /* Wait for starter to begin */
    wait_for_starter();
    NOTICE("Starting game\n");

    int i;

    while (true) {
        /* Go to lunar module */
        i = 0;
        while (!strategy_goto_avoid(robot, 780, 1340, 45)) {
            DEBUG("Try #%d", i);
            i++;
        }

        /* Push lunar module */
        trajectory_d_rel(&robot->traj, 100.);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);
        trajectory_d_rel(&robot->traj, -100.);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

        /* Go back to home */
        i = 0;
        while (!strategy_goto_avoid(robot, 600, 200, 90)) {
            DEBUG("Try #%d", i);
            i++;
        }

        DEBUG("Game ended!\nInsert coin to play more.\n");
        chThdSleepSeconds(1);

        wait_for_starter();
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, STRATEGY_STACKSIZE);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
