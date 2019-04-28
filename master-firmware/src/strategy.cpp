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
#include "strategy/actions.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy/score_counter.h"

#include "protobuf/sensors.pb.h"

static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;

static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void strategy_wait_ms(int ms);

/** Put the robot 90 degrees from the wall facing the front sensors */
static void strategy_align_front_sensors(void);

void strategy_play_game(void);

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

static void strategy_wait_ms(int ms)
{
    chThdSleepMilliseconds(ms);
}

void strategy_stop_robot(void)
{
    trajectory_stop(&robot.traj);
    strategy_wait_ms(200);
    robot.mode = BOARD_MODE_FREE;
    strategy_wait_ms(200);
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
}

static void strategy_align_front_sensors(void)
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

    trajectory_a_rel(&robot.traj, DEGREES(-alpha));
    trajectory_wait_for_end(TRAJ_FLAGS_ROTATION);
}

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags)
{
    auto map = map_server_map_lock_and_get();

    /* Compute path */
    const point_t start = {
        position_get_x_float(&robot.pos),
        position_get_y_float(&robot.pos)};
    oa_start_end_points(&map->oa, start.x, start.y, x_mm, y_mm);
    oa_process(&map->oa);

    /* Retrieve path */
    point_t* points;
    int num_points = oa_get_path(&map->oa, &points);
    DEBUG("Path to (%d, %d) computed with %d points", x_mm, y_mm, num_points);
    if (num_points <= 0) {
        WARNING("No path found!");
        strategy_stop_robot();
        map_server_map_release(map);
        return false;
    }

    /* Execute path, one waypoint at a time */
    int end_reason = 0;

    for (int i = 0; i < num_points; i++) {
        DEBUG("Going to x: %.1fmm y: %.1fmm", points[i].x, points[i].y);

        trajectory_goto_xy_abs(&robot.traj, points[i].x, points[i].y);

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
        strategy_wait_ms(200);
        trajectory_a_abs(&robot.traj, a_deg);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

        DEBUG("Goal reached successfully");
        map_server_map_release(map);

        return true;
    } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
        control_panel_set(LED_PC);
        strategy_stop_robot();
        strategy_wait_ms(100);
        strategy_stop_robot();
        control_panel_clear(LED_PC);
        WARNING("Stopping robot because opponent too close");
    } else if (end_reason == TRAJ_END_COLLISION) {
        strategy_stop_robot();
        WARNING("Stopping robot because collision detected");
    } else if (end_reason == TRAJ_END_TIMER) {
        strategy_stop_robot();
        WARNING("Stopping robot because game has ended !");
    } else {
        WARNING("Trajectory ended with reason %d", end_reason);
    }

    map_server_map_release(map);
    return false;
}

bool strategy_goto_avoid_retry(int x_mm, int y_mm, int a_deg, int traj_end_flags, int num_retries)
{
    bool finished = false;
    int counter = 0;

    while (!finished) {
        DEBUG("Try #%d", counter);
        finished = strategy_goto_avoid(x_mm, y_mm, a_deg, traj_end_flags);
        counter++;

        // Exit when maximum number of retries is reached
        // Negative number of retries means infinite number of retries
        if (num_retries >= 0 && counter > num_retries) {
            break;
        }
    }

    return finished;
}

bool strategy_puck_is_picked(void)
{
    return motor_get_current("pump-1") > config_get_scalar("master/arms/right/gripper/current_thres")
        && motor_get_current("pump-2") > config_get_scalar("master/arms/right/gripper/current_thres");
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState& state)
    {
        NOTICE("Indexing arms!");

        // const char* motors[3] = {"theta-1", "theta-2", "theta-3"};
        // const float speeds[3] = {0.15, 0.12, 0.06};
        const float directions[3] = {-1, -1, 1};
        float offsets[3];

        // arm_motors_index(motors, directions, speeds, offsets);

        // set index when user presses color button, so indexing is done manually
        offsets[0] = motor_get_position("theta-1");
        offsets[1] = motor_get_position("theta-2");
        offsets[2] = motor_get_position("theta-3");
        arm_compute_offsets(directions, offsets);

        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q1"), offsets[0]);
        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q2"), offsets[1]);
        parameter_scalar_set(PARAMETER("master/arms/right/offsets/q3"), offsets[2]);

        strategy_wait_ms(500);
        wait_for_color_selection();

        state.arms_are_indexed = true;
        return true;
    }
};

struct RetractArms : actions::RetractArms {
    enum strat_color_t m_color;

    RetractArms(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        NOTICE("Retracting arms!");

        manipulator_gripper_set(GRIPPER_OFF);
        manipulator_goto(MANIPULATOR_RETRACT);

        state.has_puck = false;
        state.arms_are_deployed = false;
        return true;
    }
};

struct TakePuck : actions::TakePuck {
    enum strat_color_t m_color;

    TakePuck(enum strat_color_t color, size_t id)
        : actions::TakePuck(id)
        , m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        float x, y, a;
        if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
            x = MIRROR_X(m_color, pucks[puck_id].pos_x_mm - 170);
            y = pucks[puck_id].pos_y_mm + MIRROR(m_color, 50);
            a = MIRROR_A(m_color, 180);
        } else {
            x = MIRROR_X(m_color, pucks[puck_id].pos_x_mm) - 50;
            y = pucks[puck_id].pos_y_mm - 260;
            a = MIRROR_A(m_color, -90);
        }

        if (!strategy_goto_avoid(x, y, a, TRAJ_FLAGS_ALL)) {
            return false;
        }

        if (pucks[puck_id].orientation == PuckOrientiation_VERTICAL) {
            strategy_align_front_sensors();
        }

        state.arms_are_deployed = true;
        manipulator_gripper_set(GRIPPER_ACQUIRE);

        if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
            manipulator_goto(MANIPULATOR_PICK_HORZ);
        } else {
            manipulator_goto(MANIPULATOR_PICK_VERT);
        }
        strategy_wait_ms(500);
        manipulator_goto(MANIPULATOR_LIFT_HORZ);

        state.puck_available[puck_id] = false;

        if (!strategy_puck_is_picked()) {
            manipulator_gripper_set(GRIPPER_OFF);
            return false;
        }

        state.has_puck = true;
        state.has_puck_color = pucks[puck_id].color;
        return true;
    }
};

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

        if (!strategy_goto_avoid(x, y, a, TRAJ_FLAGS_ALL)) {
            return false;
        }
        manipulator_gripper_set(GRIPPER_RELEASE);
        strategy_wait_ms(100);

        manipulator_gripper_set(GRIPPER_OFF);

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

        if (!strategy_goto_avoid(x, 330, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        manipulator_goto(MANIPULATOR_DEPLOY_FULLY);
        trajectory_d_rel(&robot.traj, -30);

        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

        trajectory_a_rel(&robot.traj, MIRROR(m_color, 13));
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

        if (!strategy_goto_avoid(x, 400, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        manipulator_goto(MANIPULATOR_PICK_GOLDONIUM);

        if (!strategy_goto_avoid(x, 330, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        manipulator_gripper_set(GRIPPER_ACQUIRE);
        trajectory_d_rel(&robot.traj, -27);
        strategy_wait_ms(1500);

        if (!strategy_puck_is_picked()) {
            manipulator_gripper_set(GRIPPER_OFF);
            trajectory_d_rel(&robot.traj, 80);
            trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
            return false;
        }

        manipulator_goto(MANIPULATOR_LIFT_GOLDONIUM);
        strategy_wait_ms(500);
        manipulator_gripper_set(GRIPPER_OFF);
        trajectory_d_rel(&robot.traj, 80);
        trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

        state.goldonium_in_house = false;
        state.has_goldonium = true;
        return true;
    }
};

void strategy_shutdown_endgame(void)
{
    manipulator_gripper_set(GRIPPER_OFF);
}

void strategy_order_play_game(enum strat_color_t color, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    goap::Goal<RobotState>* goals[] = {};

    IndexArms index_arms;
    RetractArms retract_arms(color);

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
    strategy_auto_position(MIRROR_X(color, 250), 450, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
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

void strategy_chaos_play_game(enum strat_color_t color, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    ClassifyStartPucksGoal classify_start_pucks_goal;
    AcceleratorGoal accelerator_goal;
    TakeGoldoniumGoal take_goldonium_goal;
    ClassifyBluePucksGoal classify_blue_pucks_goal;
    goap::Goal<RobotState>* goals[] = {
        &classify_start_pucks_goal,
        &accelerator_goal,
        &take_goldonium_goal,
        &classify_blue_pucks_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    TakePuck take_pucks[] = {{color, 0}, {color, 1}, {color, 2}, {color, 6}};
    DepositPuck deposit_puck[] = {{color, 0}, {color, 1}, {color, 2}};
    LaunchAccelerator launch_accelerator(color);
    TakeGoldonium take_goldonium(color);

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
    strategy_auto_position(MIRROR_X(color, 250), 450, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
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
    enum strat_color_t color = wait_for_color_selection();
    map_server_start(color);
    score_counter_start();

    if (config_get_boolean("master/is_main_robot")) {
        NOTICE("First, Order...");
        strategy_order_play_game(color, state);
    } else {
        NOTICE("Then, Chaos!");
        strategy_chaos_play_game(color, state);
    }
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, NULL);
}
