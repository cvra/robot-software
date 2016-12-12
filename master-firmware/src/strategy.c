#include <ch.h>
#include <hal.h>

#include <error/error.h>
#include "priorities.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "robot_parameters.h"
#include "base/base_controller.h"
#include "base/map.h"
#include "main.h"

#include "strategy.h"

#define STRATEGY_STACKSIZE 1024

static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
void strategy_play_game(void* robot);

static void wait_for_starter(void)
{
    /* Wait for a rising edge */
    while (palReadPad(GPIOE, GPIOE_STARTER)) {
        chThdSleepMilliseconds(10);
    }
    while (!palReadPad(GPIOE, GPIOE_STARTER)) {
        chThdSleepMilliseconds(10);
    }
}

static void wait_for_autoposition_signal(void)
{
    wait_for_starter();
}

void strategy_goto_avoid(struct _robot* robot, int x_mm, int y_mm, int a_deg, int num_retries)
{
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
    NOTICE("Path to (%d, %d) computed with %d points\r\n", x_mm, y_mm, num_points);

    /* Execute path, one waypoint at a time */
    for (int j = 0; j < num_retries; j++) {
        NOTICE("Try #%d", j);
        int end_reason = 0;

        for (int i = 0; i < num_points; i++) {
            NOTICE("Going to x: %.1fmm y: %.1fmm\r\n", points[i].x, points[i].y);

            trajectory_goto_xy_abs(&robot->traj, points[i].x, points[i].y);
            end_reason = trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED | TRAJ_END_OPPONENT_NEAR);

            if (end_reason == TRAJ_END_OPPONENT_NEAR) {
                break;
            }
        }

        if (end_reason == TRAJ_END_GOAL_REACHED) {
            break;
        } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
            trajectory_hardstop(&robot->traj);
            rs_set_distance(&robot->rs, 0);
            rs_set_angle(&robot->rs, 0);
            NOTICE("Stopping robot because opponent too close");

            /* Create obstacle at opponent position */
            float beacon_signal[3];
            messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");
            messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

            float x_opp, y_opp;
            beacon_cartesian_convert(&robot->pos, 1000 * beacon_signal[1], beacon_signal[2], &x_opp, &y_opp);
            strategy_set_opponent_obstacle(x_opp, y_opp, robot->opponent_size * 1.25, robot->robot_size);

            /* Query path around opponent */
            oa_reset();
            const point_t current_pos = {
                    position_get_x_s16(&robot->pos),
                    position_get_y_s16(&robot->pos)
                };
            oa_start_end_points(current_pos.x, current_pos.y, x_mm, y_mm);
            oa_process();
            num_points = oa_get_path(&points);

            NOTICE("Retrying new path around opponent with %d points", num_points);
            continue;
        }
    }

    trajectory_a_abs(&robot->traj, a_deg);
    trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);
}


void strategy_play_game(void* _robot)
{
    struct _robot* robot = (struct _robot*)_robot;
    enum strat_color_t color = YELLOW;

    /* Initialize map and path planner */
    map_init(robot->robot_size);

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot\n");
    strategy_auto_position(600, 200, 90, robot->robot_size, color, robot, &bus);
    NOTICE("Robot positioned at x: 600[mm], y: 200[mm], a: 90[deg]\n");

    /* Wait for starter to begin */
    wait_for_starter();
    NOTICE("Starting game\n");

    /* Go to lunar module */
    strategy_goto_avoid(robot, 780, 1340, 45, 10);

    /* Push lunar module */
    trajectory_d_rel(&robot->traj, 100.);
    trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);
    trajectory_d_rel(&robot->traj, -100.);
    trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

    /* Go back to home */
    strategy_goto_avoid(robot, 900, 200, 0, 10);

    while (true) {
        DEBUG("Game ended!\nInsert coin to play more.\n");
        chThdSleepSeconds(1);
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, STRATEGY_STACKSIZE);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
