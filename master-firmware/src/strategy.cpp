#include <ch.h>
#include <hal.h>
#include <array>

#include "electron_starter.hpp"

#include <aversive/blocking_detection_manager/blocking_detection_manager.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <error/error.h>
#include <goap/goap.hpp>
#include <timestamp/timestamp.h>

#include "priorities.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/motor_helpers.h"
#include "robot_helpers/arm_helpers.h"
#include "base/base_controller.h"
#include "base/base_helpers.h"
#include "base/map.h"
#include "base/map_server.h"
#include "manipulator/manipulator_thread.h"
#include "config.h"
#include "control_panel.h"
#include "main.h"

#include "strategy.h"
#include "strategy/goals.h"
#include "strategy/score_counter.h"
#include "strategy_impl/base.h"
#include "strategy_impl/game.h"
#include "strategy/state.h"

static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;

static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void wait_for_user_input(void);
void strategy_play_game(void);

static void strategy_log(const char* log)
{
    NOTICE(log);
}

static void strategy_wait_ms(int ms)
{
    chThdSleepMilliseconds(ms);
}

static void strategy_rotate(void* ctx, int relative_angle_deg)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;
    trajectory_a_rel(&strat->robot->traj, relative_angle_deg);
    trajectory_wait_for_end(TRAJ_FLAGS_ROTATION);
}

static void strategy_forward(void* ctx, int relative_distance_mm)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;
    trajectory_d_rel(&strat->robot->traj, relative_distance_mm);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
}

static bool strategy_goto_xya(void* ctx, int x_mm, int y_mm, int a_deg)
{
    strategy_context_t* strat = (strategy_context_t*)ctx;
    return strategy_goto_avoid(strat, x_mm, y_mm, a_deg, TRAJ_FLAGS_ALL);
}

static strategy_context_t strategy = {
    /*robot*/ &robot,
    /*color*/ YELLOW,
    /*log*/ strategy_log,
    /*wait_ms*/ strategy_wait_ms,
    /*wait_for_user_input*/ wait_for_user_input,
    /*forward*/ strategy_forward,
    /*rotate*/ strategy_rotate,
    /*goto_xya*/ strategy_goto_xya,
    /*manipulator_goto*/ manipulator_goto,
    /*manipulator_disable*/ arm_turn_off,
    /*gripper_set*/ manipulator_gripper_set,
    /*puck_is_picked*/ strategy_puck_is_picked,
    /*arm_manual_index*/ arm_manual_index,
};

strategy_context_t* strategy_impl(void)
{
    return &strategy;
}

static void wait_for_user_input(void)
{
    wait_for_color_selection();
}

static enum strat_color_t wait_for_color_selection(void)
{
    strat_color_t color = YELLOW;

    while (!control_panel_button_is_pressed(BUTTON_YELLOW) && !control_panel_button_is_pressed(BUTTON_GREEN)) {
        control_panel_set(LED_YELLOW);
        control_panel_set(LED_GREEN);
        strategy_wait_ms(100);

        control_panel_clear(LED_YELLOW);
        control_panel_clear(LED_GREEN);
        strategy_wait_ms(100);
    }

    if (control_panel_button_is_pressed(BUTTON_GREEN)) {
        control_panel_clear(LED_YELLOW);
        control_panel_set(LED_GREEN);
        color = VIOLET;
        NOTICE("Color set to violet");
    } else {
        control_panel_set(LED_YELLOW);
        control_panel_clear(LED_GREEN);
        color = YELLOW;
        NOTICE("Color set to yellow");
    }

    return color;
}

static void wait_for_starter(void)
{
    const long STARTER_MIN_ARMED_TIME_MS = 1000;
    unsigned long starter_armed_time_ms;

    control_panel_clear(LED_READY);

    while (1) {
        starter_armed_time_ms = 0;

        /* Wait for a rising edge */
        while (control_panel_read(STARTER)) {
            strategy_wait_ms(10);
        }
        while (!control_panel_read(STARTER)) {
            strategy_wait_ms(10);
            starter_armed_time_ms += 10;

            /* indicate that starter is armed */
            if (starter_armed_time_ms >= STARTER_MIN_ARMED_TIME_MS) {
                control_panel_set(LED_READY);
            }
        }

        if (starter_armed_time_ms >= STARTER_MIN_ARMED_TIME_MS) {
            break;
        }
    }
}

static void wait_for_autoposition_signal(void)
{
    wait_for_color_selection();
}

bool strategy_puck_is_picked(manipulator_side_t side)
{
    bool res = true;
    if (USE_LEFT(side)) {
        res &= motor_get_current("left-pump-1") > config_get_scalar("master/arms/left/gripper/current_thres")
            && motor_get_current("left-pump-2") > config_get_scalar("master/arms/left/gripper/current_thres");
    }
    if (USE_RIGHT(side)) {
        res &= motor_get_current("right-pump-1") > config_get_scalar("master/arms/right/gripper/current_thres")
            && motor_get_current("right-pump-2") > config_get_scalar("master/arms/right/gripper/current_thres");
    }
    return res;
}

void strategy_shutdown_endgame(void)
{
    manipulator_gripper_set(BOTH, GRIPPER_OFF);
}

void strategy_order_play_game(strategy_context_t* ctx, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    InitGoal init_goal;
    GAME_GOALS_ORDER(goals, goal_names, goal_count);
    GAME_ACTIONS_ORDER(actions, action_count, ctx);

    (void)goal_names;
    (void)goal_count;

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, actions, action_count, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        messagebus_topic_publish(state_topic, &state, sizeof(state));
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(ctx->color, 250), 450, MIRROR_A(ctx->color, -90), ctx->color);

    trajectory_a_abs(&robot.traj, MIRROR_A(ctx->color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;
    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();
    trajectory_game_timer_reset();

    NOTICE("Starting game...");
    while (!trajectory_game_has_ended()) {
        for (auto goal : goals) {
            int len = planner.plan(state, *goal, actions, action_count, path, max_path_len);
            for (int i = 0; i < len; i++) {
                bool success = path[i]->execute(state);
                messagebus_topic_publish(state_topic, &state, sizeof(state));
                chThdYield();
                if (success == false) {
                    break; // Break on failure
                }
                if (trajectory_game_has_ended()) {
                    break;
                }
            }
            if (trajectory_game_has_ended()) {
                break;
            }
        }
        strategy_wait_ms(10);
    }
    strategy_shutdown_endgame();

    NOTICE("Game ended!");
    while (true) {
        strategy_wait_ms(1000);
    }
}

void strategy_chaos_play_game(strategy_context_t* ctx, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    InitGoal init_goal;
    GAME_GOALS_CHAOS(goals, goal_names, goal_count);
    GAME_ACTIONS_CHAOS(actions, action_count, ctx);

    (void)goal_names;
    (void)goal_count;

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, actions, action_count, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        messagebus_topic_publish(state_topic, &state, sizeof(state));
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(ctx->color, 250), 450, MIRROR_A(ctx->color, -90), ctx->color);

    trajectory_a_abs(&robot.traj, MIRROR_A(ctx->color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();

    trajectory_game_timer_reset();

    electron_starter_start();
    state.electron_launched = true;

    NOTICE("Starting game...");

    while (!trajectory_game_has_ended()) {
        for (auto goal : goals) {
            int len = planner.plan(state, *goal, actions, action_count, path, max_path_len);
            for (int i = 0; i < len; i++) {
                bool success = path[i]->execute(state);
                messagebus_topic_publish(state_topic, &state, sizeof(state));
                if (success == false) {
                    break; // Break on failure
                }
                if (trajectory_game_has_ended()) {
                    break;
                }
            }
            if (trajectory_game_has_ended()) {
                break;
            }
        }

        strategy_wait_ms(10);
    }
    strategy_shutdown_endgame();

    // Check that opponent didn't switch off our panel
    strategy_wait_ms(5000);

    NOTICE("Game ended!");
    while (true) {
        strategy_wait_ms(1000);
    }
}

void strategy_play_game(void* p)
{
    (void)p;
    chRegSetThreadName("strategy");

    NOTICE("Strategy starting...");

    /* Prepare state publisher */
    RobotState state = initial_state();

    static TOPIC_DECL(state_topic, RobotState);
    messagebus_advertise_topic(&bus, &state_topic.topic, "/state");

    NOTICE("Waiting for color selection...");
    strategy.color = wait_for_color_selection();
    map_server_start(strategy.color);
    score_counter_start();

    if (config_get_boolean("master/is_main_robot")) {
        NOTICE("First, Order...");
        strategy_order_play_game(&strategy, state);
    } else {
        NOTICE("Then, Chaos!");
        strategy_chaos_play_game(&strategy, state);
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, NULL);
}
