#include <ch.h>
#include <hal.h>

#include "log.h"
#include "priorities.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_parameters.h"
#include "base/base_controller.h"

#include "strategy.h"

#define STRATEGY_STACKSIZE 1024

static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
void strategy_play_game(void* robot);

static void wait_for_autoposition_signal(void)
{
    chThdSleepSeconds(10);
}

static void wait_for_starter(void)
{
    chThdSleepSeconds(10);
}


void strategy_play_game(void* _robot)
{
    struct _robot* robot = (struct _robot*)_robot;
    enum strat_color_t color = YELLOW;

    /* Autoposition robot */
    wait_for_autoposition_signal();
    log_message("Positioning robot\n");
    strategy_auto_position(
        500, 200, 90, ROBOT_SIZE_X_MM, color,
        &robot->mode, &robot->traj, &robot->pos,
        &robot->distance_bd, &robot->angle_bd);
    log_message("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]\n", 500, 200, 90);

    /* Configure robot dynamics */
    trajectory_set_acc(&robot->traj,
            acc_mm2imp(&robot->traj, 300.),
            acc_rd2imp(&robot->traj, 3.));
    trajectory_set_speed(&robot->traj,
            speed_mm2imp(&robot->traj, 200.),
            speed_rd2imp(&robot->traj, 3.));

    bd_set_thresholds(&robot->angle_bd, 12500, 1);
    bd_set_thresholds(&robot->distance_bd, 15000, 1);

    /* Wait for starter to begin */
    wait_for_starter();
    log_message("Starting game\n");

    trajectory_move_to(&robot->traj, 500, MIRROR_Y(color, 500), MIRROR_A(color, 90));
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, STRATEGY_STACKSIZE);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
