#include <ch.h>
#include <hal.h>
#include <array>

#include <error/error.h>
#include <aversive/blocking_detection_manager/blocking_detection_manager.h>

#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
#include <goap/goap.hpp>

#include "priorities.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "base/base_controller.h"
#include "base/base_helpers.h"
#include "base/map.h"
#include "base/map_server.h"
#include "scara/scara_trajectories.h"
#include "arms/arms_controller.h"
#include "arms/arm_trajectory_manager.h"
#include "arms/arm.h"
#include "config.h"
#include "control_panel.h"
#include "main.h"

#include "strategy.h"
#include "strategy/actions.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy/score_counter.h"
#include "strategy/color_sequence_server.h"
#include "timestamp/timestamp.h"

#include "protobuf/sensors.pb.h"
#include "protobuf/strategy.pb.h"

static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;

static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void strategy_wait_ms(int ms);

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
        color = BLUE;
        NOTICE("Color set to blue");
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
    wait_for_starter();
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

bool strat_check_distance_to_hand_lower_than(float expected_value)
{
    bool success = true;
    Range range;

    messagebus_topic_t* topic = messagebus_find_topic_blocking(&bus, "/hand_distance");

    if (messagebus_topic_read(topic, &range, sizeof(range))) {
        WARNING("Hand distance: %f", range.distance);
        success = (range.distance < expected_value);
    } else {
        WARNING("Hand distance sensor is not publishing");
    }

    return success;
}


void strat_scara_goto(position_3d_t pos, scara_coordinate_t system, velocity_3d_t max_vel, int watched_events)
{
    NOTICE("Moving arm to %.1f %.1f %.1f in system %d", pos.x, pos.y, pos.z, (int)system);
    scara_goto(&main_arm, pos, system, max_vel);
    arm_traj_wait_for_event(watched_events);

    position_3d_t arm_pos = scara_position(&main_arm, system);
    NOTICE("Arm at %.1f %.1f %.1f in system %d", arm_pos.x, arm_pos.y, arm_pos.z, (int)system);
}

void strat_scara_goto_blocking(position_3d_t pos, scara_coordinate_t system, velocity_3d_t max_vel)
{
    NOTICE("Moving arm to %.1f %.1f %.1f in system %d", pos.x, pos.y, pos.z, (int)system);
    scara_goto(&main_arm, pos, system, max_vel);
    arm_traj_wait_for_end();

    position_3d_t arm_pos = scara_position(&main_arm, system);
    NOTICE("Arm at %.1f %.1f %.1f in system %d", arm_pos.x, arm_pos.y, arm_pos.z, (int)system);
}

void strat_scara_push_x(float dx, scara_coordinate_t system, velocity_3d_t max_vel)
{
    const position_3d_t last_pos = scara_position(&main_arm, system);
    strat_scara_goto({last_pos.x + dx, last_pos.y, last_pos.z}, system, max_vel, ARM_READY | ARM_BLOCKED_XY);
}

void strat_scara_push_y(float dy, scara_coordinate_t system, velocity_3d_t max_vel)
{
    const position_3d_t last_pos = scara_position(&main_arm, system);
    strat_scara_goto({last_pos.x, last_pos.y + dy, last_pos.z}, system, max_vel, ARM_READY | ARM_BLOCKED_XY);
}

float safe_z(float z)
{
    return fminf(z, 300.f);
}

void strat_scara_force_shoulder_mode(shoulder_mode_t shoulder_mode)
{
    scara_control_mode_joint(&main_arm);

    main_arm.shoulder_mode = shoulder_mode;
    scara_goto(&main_arm, {170., 0., 295.}, COORDINATE_ARM, {300, 300, 300});
    strategy_wait_ms(500);

    scara_control_mode_cartesian(&main_arm);
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState& state)
    {
        NOTICE("Indexing arms!");

        /* Z axis indexing */
        cvra_arm_motor_t* z_motors[] = {
            (cvra_arm_motor_t*)main_arm.hw_interface.z_joint.args,
        };
        float z_speeds[] = {-20};
        arms_auto_index(z_motors, z_speeds, sizeof(z_speeds) / sizeof(float));

        z_motors[0]->index += config_get_scalar("master/arms/motor_offsets/z-joint");

        /* Arm indexing */
        cvra_arm_motor_t* motors[] = {
            (cvra_arm_motor_t*)main_arm.hw_interface.shoulder_joint.args,
            (cvra_arm_motor_t*)main_arm.hw_interface.elbow_joint.args,
        };
        float motor_speeds[] = {0.8, 0.8};
        arms_auto_index(motors, motor_speeds, sizeof(motor_speeds) / sizeof(float));

        motors[0]->index += config_get_scalar("master/arms/motor_offsets/shoulder-joint");
        motors[1]->index += config_get_scalar("master/arms/motor_offsets/elbow-joint");

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
        state.arms_are_deployed = false;

        scara_control_mode_joint(&main_arm);

        main_arm.shoulder_mode = MIRROR_SHOULDER(m_color, SHOULDER_BACK);
        scara_goto(&main_arm, {170., 0., 295.}, COORDINATE_ARM, {300, 300, 300});
        strategy_wait_ms(500);

        scara_goto(&main_arm, {20.f, MIRROR(m_color, 90.f), 295.f}, COORDINATE_ROBOT, {300, 300, 300});
        strategy_wait_ms(500);

        scara_control_mode_cartesian(&main_arm);

        return true;
    }
};

struct TakePuck : actions::TakePuck {
    enum strat_color_t m_color;

    TakePuck(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        NOTICE("Going for the puck!");

        if (!strategy_goto_avoid(MIRROR_X(m_color, 250), 450, MIRROR_A(m_color, 180), TRAJ_FLAGS_ALL)) {
            NOTICE("Failed to reach position!");
            return false;
        }

        const position_3d_t start_pos = scara_position(&main_arm, COORDINATE_TABLE);
        strat_scara_goto({MIRROR_X(m_color, 500), 450, start_pos.z}, COORDINATE_TABLE, {300, 300, 300}, ARM_READY);

        strat_scara_goto({MIRROR_X(m_color, 500), 450, 25}, COORDINATE_TABLE, {300, 300, 300}, ARM_READY);

        bool puck_is_present = strat_check_distance_to_hand_lower_than(0.05f);
        if (puck_is_present) {
            hand_set_pump(&main_hand, PUMP_ON);
            strategy_wait_ms(500);
        }

        strat_scara_goto({MIRROR_X(m_color, 500), 450, start_pos.z}, COORDINATE_TABLE, {300, 300, 300}, ARM_READY);

        state.arms_are_deployed = true;
        state.has_puck = true;

        return true;
    }
};

void strategy_order_play_game(enum strat_color_t color, RobotState& state)
{
    // messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    FirstPuckGoal first_puck_goal;
    goap::Goal<RobotState>* goals[] = {
        &first_puck_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    TakePuck take_puck(color);

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    goap::Action<RobotState>* actions[] = {
        &index_arms,
        &retract_arms,
        &take_puck,
    };

    const auto action_count = sizeof(actions) / sizeof(actions[0]);

    wrist_set_horizontal(&main_wrist);

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, actions, action_count, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        // messagebus_topic_publish(state_topic, &state, sizeof(state));
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
                // messagebus_topic_publish(state_topic, &state, sizeof(state));
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

    NOTICE("Game ended!");
    while (true) {
        strategy_wait_ms(1000);
    }
}

void strategy_chaos_play_game(enum strat_color_t color, RobotState& state)
{
    // messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;
    FirstPuckGoal first_puck_goal;
    goap::Goal<RobotState>* goals[] = {
        &first_puck_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    TakePuck take_puck(color);

    const int max_path_len = 10;
    goap::Action<RobotState>* path[max_path_len] = {nullptr};

    goap::Action<RobotState>* actions[] = {
        &index_arms,
        &retract_arms,
        &take_puck,
    };

    const auto action_count = sizeof(actions) / sizeof(actions[0]);

    wrist_set_horizontal(&main_wrist);

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, actions, action_count, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        // messagebus_topic_publish(state_topic, &state, sizeof(state));
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(color, 250), 750, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();

    trajectory_game_timer_reset();

    NOTICE("Moving out of Order's way. Time for Chaos!");
    trajectory_d_rel(&robot.traj, -400);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    NOTICE("Starting game...");

    while (!trajectory_game_has_ended()) {
        for (auto goal : goals) {
            int len = planner.plan(state, *goal, actions, action_count, path, max_path_len);
            for (int i = 0; i < len; i++) {
                bool success = path[i]->execute(state);
                // messagebus_topic_publish(state_topic, &state, sizeof(state));
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
    RobotState state;

    // static messagebus_topic_t state_topic;
    // static MUTEX_DECL(state_lock);
    // static CONDVAR_DECL(state_condvar);
    // static RobotState state_topic_content;
    // messagebus_topic_init(&state_topic, &state_lock, &state_condvar, &state_topic_content, sizeof(state));
    // messagebus_advertise_topic(&bus, &state_topic, "/state");

    NOTICE("Waiting for color selection...");
    enum strat_color_t color = wait_for_color_selection();
    map_server_start(color);
    score_counter_start();
    color_sequence_server_start();

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
