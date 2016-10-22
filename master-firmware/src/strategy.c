#include <ch.h>
#include <hal.h>

#include "log.h"
#include "priorities.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_parameters.h"
#include "base/base_controller.h"

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


void strategy_play_game(void* _robot)
{
    struct _robot* robot = (struct _robot*)_robot;
    enum strat_color_t color = YELLOW;

    /* Autoposition robot */
    wait_for_autoposition_signal();
    log_message("Positioning robot\n");
    strategy_auto_position(
        900, 200, 90, ROBOT_SIZE_X_MM, color,
        &robot->mode, &robot->traj, &robot->pos,
        &robot->distance_bd, &robot->angle_bd);
    log_message("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]\n", 900, 200, 90);

    /* Configure robot dynamics */
    trajectory_set_acc(&robot->traj,
            acc_mm2imp(&robot->traj, 300.),
            acc_rd2imp(&robot->traj, 3.));
    trajectory_set_speed(&robot->traj,
            speed_mm2imp(&robot->traj, 200.),
            speed_rd2imp(&robot->traj, 3.));

    bd_set_thresholds(&robot->angle_bd, 12500, 1);
    bd_set_thresholds(&robot->distance_bd, 15000, 1);

    /* Add obstacles */
    poly_t *obstacle = oa_new_poly(4);
    oa_poly_set_point(obstacle, 500, 400, 0);
    oa_poly_set_point(obstacle, 500, 700, 1);
    oa_poly_set_point(obstacle, 800, 700, 2);
    oa_poly_set_point(obstacle, 800, 400, 3);

    /* Wait for starter to begin */
    wait_for_starter();
    log_message("Starting game\n");

    /* Go somewhere avoiding obstacles */
    oa_reset();
    oa_start_end_points(900, 200, 200, 600);
    oa_process();

    point_t *points;
    int num_points = oa_get_path(&points);
    log_message("Path computed with %d points\r\n", num_points);

    for (int i = 0; i < num_points; i++) {
        log_message("Going to x: %.1fmm y: %.1fmm\r\n", points[i].x, points[i].y);
        trajectory_goto_xy_abs(&robot->traj, points[i].x, points[i].y);
        trajectory_wait_for_finish(&robot->traj);
    }

    while (true) {
        log_message("Game ended!\nInsert coin to play more.\n");
        chThdSleepSeconds(1);
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, STRATEGY_STACKSIZE);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
