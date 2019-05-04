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
#include "strategy_impl.h"

static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;

static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void wait_for_user_input(void);
void strategy_play_game(void);

static void strategy_wait_ms(int ms)
{
    chThdSleepMilliseconds(ms);
}

static strategy_context_t strategy = {
    /*robot*/ &robot,
    /*color*/ YELLOW,
    /*wait_ms*/ strategy_wait_ms,
    /*wait_for_user_input*/ wait_for_user_input,
    /*manipulator_goto*/ manipulator_goto,
    /*gripper_set*/ manipulator_gripper_set,
    /*puck_is_picked*/ strategy_puck_is_picked,
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

bool strategy_puck_is_picked(void)
{
    return motor_get_current("pump-1") > config_get_scalar("master/arms/right/gripper/current_thres")
        && motor_get_current("pump-2") > config_get_scalar("master/arms/right/gripper/current_thres");
}

struct DepositPuck : actions::DepositPuck {
    enum strat_color_t m_color;

    DepositPuck(enum strat_color_t color, size_t zone_id)
        : actions::DepositPuck(zone_id)
        , m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        float x = MIRROR_X(m_color, areas[zone_id].pos_x_mm);
        float y = areas[zone_id].pos_y_mm - MIRROR(m_color, 50);
        float a = MIRROR_A(m_color, 0);

        if (!strategy_goto_avoid(&strategy, x, y, a, TRAJ_FLAGS_ALL)) {
            return false;
        }
        manipulator_gripper_set(RIGHT, GRIPPER_RELEASE);
        strategy_wait_ms(100);

        manipulator_gripper_set(RIGHT, GRIPPER_OFF);

        pucks_in_area++;
        state.has_puck = false;
        state.classified_pucks[areas[zone_id].color]++;
        state.arms_are_deployed = true;
        return true;
    }
};

struct LaunchAccelerator : actions::LaunchAccelerator {
    enum strat_color_t m_color;

    LaunchAccelerator(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        float x = (m_color == VIOLET) ? 1695 : 1405;

        if (!strategy_goto_avoid(&strategy, x, 330, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        manipulator_goto(RIGHT, MANIPULATOR_DEPLOY_FULLY);
        trajectory_d_rel(&robot.traj, -30);

        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

        trajectory_a_rel(&robot.traj, MIRROR(m_color, 20));
        trajectory_wait_for_end(TRAJ_FLAGS_ROTATION);
        trajectory_d_rel(&robot.traj, 40);
        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
        state.accelerator_is_done = true;
        return true;
    }
};

struct TakeGoldonium : actions::TakeGoldonium {
    enum strat_color_t m_color;

    TakeGoldonium(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        float x = (m_color == VIOLET) ? 2275 : 825;

        if (!strategy_goto_avoid(&strategy, x, 400, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        manipulator_goto(RIGHT, MANIPULATOR_PICK_GOLDONIUM);

        if (!strategy_goto_avoid(&strategy, x, 330, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        manipulator_gripper_set(RIGHT, GRIPPER_ACQUIRE);
        trajectory_d_rel(&robot.traj, -27);
        strategy_wait_ms(1500);

        if (!strategy_puck_is_picked()) {
            manipulator_gripper_set(RIGHT, GRIPPER_OFF);
            trajectory_d_rel(&robot.traj, 80);
            trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
            return false;
        }

        manipulator_goto(RIGHT, MANIPULATOR_LIFT_GOLDONIUM);
        strategy_wait_ms(500);
        manipulator_gripper_set(RIGHT, GRIPPER_OFF);

        trajectory_d_rel(&robot.traj, 80);
        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

        state.goldonium_in_house = false;
        state.has_goldonium = true;
        return true;
    }
};

void strategy_shutdown_endgame(void)
{
    manipulator_gripper_set(BOTH, GRIPPER_OFF);
}

void strategy_order_play_game(RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    goap::Goal<RobotState>* goals[] = {};

    IndexArms index_arms(&strategy);
    RetractArms retract_arms(&strategy);

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    goap::Action<RobotState>* actions[] = {
        &index_arms,
        &retract_arms,
    };

    const auto action_count = sizeof(actions) / sizeof(actions[0]);

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
    strategy_auto_position(MIRROR_X(strategy.color, 250), 450, MIRROR_A(strategy.color, -90), strategy.color);

    trajectory_a_abs(&robot.traj, MIRROR_A(strategy.color, 180));
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

void strategy_chaos_play_game(RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    ClassifyStartPucksGoal classify_start_pucks_goal;
    AcceleratorGoal accelerator_goal;
    TakeGoldoniumGoal take_goldonium_goal;
    ClassifyBluePucksGoal classify_blue_pucks_goal;
    ClassifyGreenPucksGoal classify_green_pucks_goal;
    ClassifyRedPucksGoal classify_red_pucks_goal;
    goap::Goal<RobotState>* goals[] = {
        &classify_start_pucks_goal,
        &accelerator_goal,
        &take_goldonium_goal,
        &classify_blue_pucks_goal,
        &classify_green_pucks_goal,
        &classify_red_pucks_goal,
    };

    IndexArms index_arms(&strategy);
    RetractArms retract_arms(&strategy);
    TakePuck take_pucks[] = {{&strategy, 0}, {&strategy, 1}, {&strategy, 2}, {&strategy, 3}, {&strategy, 4}, {&strategy, 5}, {&strategy, 6}, {&strategy, 7}, {&strategy, 8}, {&strategy, 9}, {&strategy, 10}, {&strategy, 11}};
    DepositPuck deposit_puck[] = {{strategy.color, 0}, {strategy.color, 1}, {strategy.color, 2}, {strategy.color, 3}, {strategy.color, 4}};
    LaunchAccelerator launch_accelerator(strategy.color);
    TakeGoldonium take_goldonium(strategy.color);

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    goap::Action<RobotState>* actions[] = {
        &index_arms,
        &retract_arms,
        &take_pucks[0],
        &take_pucks[1],
        &take_pucks[2],
        &take_pucks[3],
        &deposit_puck[0],
        &deposit_puck[1],
        &deposit_puck[2],
        &launch_accelerator,
        &take_goldonium,
    };

    const auto action_count = sizeof(actions) / sizeof(actions[0]);

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
    strategy_auto_position(MIRROR_X(strategy.color, 250), 450, MIRROR_A(strategy.color, -90), strategy.color);

    trajectory_a_abs(&robot.traj, MIRROR_A(strategy.color, 180));
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
        strategy_order_play_game(state);
    } else {
        NOTICE("Then, Chaos!");
        strategy_chaos_play_game(state);
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, NULL);
}
