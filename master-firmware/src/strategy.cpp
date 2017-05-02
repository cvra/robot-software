#include <ch.h>
#include <hal.h>

#include "can/rocket_driver.h"
#include <error/error.h>
#include <timestamp/timestamp.h>
#include <blocking_detection_manager/blocking_detection_manager.h>
#include <trajectory_manager/trajectory_manager_utils.h>
#include <obstacle_avoidance/obstacle_avoidance.h>
#include <goap/goap.hpp>

#include "priorities.h"
#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/strategy_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "base/base_controller.h"
#include "base/map.h"
#include "scara/scara_trajectories.h"
#include "arms/arms_controller.h"
#include "config.h"
#include "main.h"

#include "strategy.h"


static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
void strategy_play_game(void);

#define BUTTON_IS_PRESSED(port, io) (palReadPad(port, io) == false) // Active low

static enum strat_color_t wait_for_color_selection(void)
{
    strat_color_t color = YELLOW;

    while (!BUTTON_IS_PRESSED(GPIOF, GPIOF_BTN_YELLOW) && !BUTTON_IS_PRESSED(GPIOF, GPIOF_BTN_GREEN)) {
        palSetPad(GPIOF, GPIOF_LED_YELLOW_1);
        palSetPad(GPIOF, GPIOF_LED_GREEN_1);
        chThdSleepMilliseconds(100);

        palClearPad(GPIOF, GPIOF_LED_YELLOW_1);
        palClearPad(GPIOF, GPIOF_LED_GREEN_1);
        chThdSleepMilliseconds(100);
    }

    if (BUTTON_IS_PRESSED(GPIOF, GPIOF_BTN_GREEN)) {
        palClearPad(GPIOF, GPIOF_LED_YELLOW_1);
        palSetPad(GPIOF, GPIOF_LED_GREEN_1);
        color = BLUE;
        NOTICE("Color set to blue");
    } else {
        palSetPad(GPIOF, GPIOF_LED_YELLOW_1);
        palClearPad(GPIOF, GPIOF_LED_GREEN_1);
        color = YELLOW;
        NOTICE("Color set to yellow");
    }

    return color;
}

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

void strategy_stop_robot(void)
{
    trajectory_hardstop(&robot.traj);
    chThdSleepMilliseconds(200);
}

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags)
{
    /* Create obstacle at opponent position */
    beacon_signal_t beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");
    messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));

    // only consider recent beacon signal
    if (timestamp_duration_s(beacon_signal.timestamp, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
        float x_opp, y_opp;
        beacon_cartesian_convert(&robot.pos, 1000 * beacon_signal.distance, beacon_signal.heading, &x_opp, &y_opp);
        map_update_opponent_obstacle(x_opp, y_opp, robot.opponent_size * 1.25, robot.robot_size);
    }

    /* Compute path */
    oa_reset();
    const point_t start = {
            position_get_x_float(&robot.pos),
            position_get_y_float(&robot.pos)
        };
    oa_start_end_points(start.x, start.y, x_mm, y_mm);
    oa_process();

    /* Retrieve path */
    point_t *points;
    int num_points = oa_get_path(&points);
    DEBUG("Path to (%d, %d) computed with %d points", x_mm, y_mm, num_points);
    if (num_points <= 0) {
        WARNING("No path found!");
        strategy_stop_robot();
        return false;
    }

    /* Execute path, one waypoint at a time */
    int end_reason = 0;

    for (int i = 0; i < num_points; i++) {
        DEBUG("Going to x: %.1fmm y: %.1fmm", points[i].x, points[i].y);

        trajectory_goto_forward_xy_abs(&robot.traj, points[i].x, points[i].y);
        end_reason = trajectory_wait_for_end(traj_end_flags);

        if (end_reason != TRAJ_END_GOAL_REACHED) {
            break;
        }
    }

    if (end_reason == TRAJ_END_GOAL_REACHED) {
        trajectory_a_abs(&robot.traj, a_deg);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

        DEBUG("Goal reached successfully");

        return true;
    } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
        palSetPad(GPIOF, GPIOF_LED_PC_ERROR);
        strategy_stop_robot();
        chThdSleepMilliseconds(100);
        palClearPad(GPIOF, GPIOF_LED_PC_ERROR);
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

enum Location {
    Other=0,
    Cylinder0,
    Cylinder1,
    Cylinder2,
    Cylinder3,
    Cylinder4,
    Cylinder5,

};

struct DebraState {
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool has_moved{false};
    bool location_accessible[7]{true, false, true, true, true, true, true};
    bool cylinder_present[6]{false, true, true, true, true, true};
    unsigned cylinder_count{0};
    enum Location near_location{Other};
};

struct IndexArms : public goap::Action<DebraState> {
    bool can_run(DebraState state)
    {
        (void) state;
        return true;
    }

    DebraState plan_effects(DebraState state)
    {
        state.arms_are_indexed = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Indexing arms!");

        const char* z_names[2] = {"left-z", "right-z"};
        int z_dirs[2] = {-1, -1};
        float z_speeds[2] = {20, 20};
        float z_indexes[2];
        arms_auto_index(z_names, z_dirs, z_speeds, 2, z_indexes);

        arms_set_motor_index(left_arm.z_args, z_indexes[0] + config_get_scalar("master/arms/motor_offsets/left-z"));
        arms_set_motor_index(right_arm.z_args, z_indexes[1] + config_get_scalar("master/arms/motor_offsets/right-z"));

        const char* motor_names[6] = {"left-shoulder", "left-elbow", "left-wrist", "right-shoulder", "right-elbow", "right-wrist"};
        int motor_dirs[6] = {1, 1, 1, -1, -1, -1};
        float motor_speeds[6] = {0.8, 0.8, 4.0, 0.8, 0.8, 4.0};
        float motor_indexes[6];
        arms_auto_index(motor_names, motor_dirs, motor_speeds, 6, motor_indexes);

        arms_set_motor_index(left_arm.shoulder_args, motor_indexes[0] + config_get_scalar("master/arms/motor_offsets/left-shoulder"));
        arms_set_motor_index(left_arm.elbow_args, motor_indexes[1] + config_get_scalar("master/arms/motor_offsets/left-elbow"));
        arms_set_motor_index(left_arm.wrist_args, motor_indexes[2] + config_get_scalar("master/arms/motor_offsets/left-wrist"));

        arms_set_motor_index(right_arm.shoulder_args, motor_indexes[3] + config_get_scalar("master/arms/motor_offsets/right-shoulder"));
        arms_set_motor_index(right_arm.elbow_args, motor_indexes[4] + config_get_scalar("master/arms/motor_offsets/right-elbow"));
        arms_set_motor_index(right_arm.wrist_args, motor_indexes[5] + config_get_scalar("master/arms/motor_offsets/right-wrist"));

        state.arms_are_indexed = true;
        return true;
    }
};

struct RetractArms : public goap::Action<DebraState> {
    bool can_run(DebraState state)
    {
        return state.arms_are_indexed;
    }

    DebraState plan_effects(DebraState state)
    {
        state.arms_are_deployed = false;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Retracting arms!");
        scara_goto(&left_arm, -110, 60, 120, COORDINATE_ROBOT, 1.);
        scara_goto(&right_arm, 110, -60, 120, COORDINATE_ROBOT, 1.);
        chThdSleepSeconds(1.);
        state.arms_are_deployed = false;
        return true;
    }
};

struct CollectCylinderRocketBody : public goap::Action<DebraState> {
    int m_x_mm, m_y_mm, m_a_deg;
    enum strat_color_t m_color;

    CollectCylinderRocketBody(enum strat_color_t color)
        : m_color(color)
    {
        m_x_mm = MIRROR_X(m_color, 1200);
        m_y_mm = 400;
        m_a_deg = MIRROR_A(m_color, 0);
    }

    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed;
    }

    DebraState plan_effects(DebraState state)
    {
        state.cylinder_count++;
        state.arms_are_deployed = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Goto location near rocket body full: %dmm %dmm %ddeg", m_x_mm, m_y_mm, m_a_deg);
        state.near_location = Other;
        if (!strategy_goto_avoid(m_x_mm, m_y_mm, m_a_deg, TRAJ_FLAGS_ALL)) {
            return false;
        }

        scara_t* arm;

        if (m_color == YELLOW) {
            arm = &left_arm;
        } else {
            arm = &right_arm;
        }

        NOTICE("Collecting cylinder rocket body");

        scara_trajectory_append_point_with_length(&arm->trajectory, MIRROR_X(m_color, 1300), 120, 120, COORDINATE_TABLE, 2., arm->length[0], arm->length[1]);

        for (int i = 0; i < 4; i++) {
            scara_trajectory_init(&arm->trajectory);
            scara_trajectory_append_point_with_length(&arm->trajectory, MIRROR_X(m_color, 1300), 120, 20, COORDINATE_TABLE, 2., arm->length[0], arm->length[1]);
            scara_trajectory_append_point_with_length(&arm->trajectory, MIRROR_X(m_color, 1200), 120, 20, COORDINATE_TABLE, 1., arm->length[0], arm->length[1]);
            scara_trajectory_append_point_with_length(&arm->trajectory, MIRROR_X(m_color, 1150), 120, 20, COORDINATE_TABLE, 1., arm->length[0], arm->length[1]);
            scara_trajectory_append_point_with_length(&arm->trajectory, MIRROR_X(m_color, 1100), 120, 20, COORDINATE_TABLE, 1., arm->length[0], arm->length[1]);
            scara_do_trajectory(arm, &arm->trajectory);
            chThdSleepSeconds(8);
        }

        state.cylinder_count++;
        state.arms_are_deployed = true;
        return true;
    }
};

struct InitGoal : goap::Goal<DebraState> {
    bool is_reached(DebraState state)
    {
        return state.arms_are_deployed == false;
    }
};

struct GameGoal : goap::Goal<DebraState> {
    bool is_reached(DebraState state)
    {
        return (state.cylinder_count > 0) && !state.arms_are_deployed;
    }
};

void strategy_debra_play_game(void)
{
    /* Wait for color selection */
    enum strat_color_t color = wait_for_color_selection();

    int len;
    InitGoal init_goal;
    IndexArms index_arms;
    RetractArms retract_arms;
    CollectCylinderRocketBody collect_rocket_body(color);

    DebraState state;

    const int max_path_len = 10;
    goap::Action<DebraState> *path[max_path_len] = {nullptr};

    goap::Action<DebraState> *actions[] = {
        &index_arms,
        &retract_arms,
        &collect_rocket_body,
    };

    goap::Planner<DebraState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    for (size_t i = 0; i < 4; i++) {
        hand_set_finger(&right_hand, i, FINGER_RETRACTED);
    }

    wait_for_autoposition_signal();
    NOTICE("Getting arms ready...");
    len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    // First alignment
    strategy_auto_position(MIRROR_X(color, 300), 200, MIRROR_A(color, -90), color);
    robot.pos.pos_d.y += 382;
    robot.pos.pos_s16.y += 382;

    // Second alignement only in y at starting area
    strategy_goto_avoid_retry(MIRROR_X(color, 890), 200, MIRROR_A(color, -90), TRAJ_END_GOAL_REACHED, -1);
    strategy_align_y(170);
    trajectory_a_abs(&robot.traj, MIRROR_A(color, 90));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();
    trajectory_game_timer_reset();
    rocket_program_launch_time(GAME_DURATION + 1);

    NOTICE("Starting game");
    GameGoal game_goal;
    while (true) {
        len = planner.plan(state, game_goal, path, max_path_len);
        NOTICE("Plan length: %d", len);
        if (len > 0) {
            bool success = true;
            for (int i = 0; i < len; i++) {
                success &= path[i]->execute(state);
                if (!success) {
                    NOTICE("Action failed, requesting new plan...");
                    break;
                }
            }
            if (success) {
                NOTICE("Goal successfully achieved, exiting.");
                break;
            }
        } else {
            NOTICE("No valid plan found, waiting...");
            chThdSleepSeconds(1);
        }
    }
}

void strategy_sandoi_play_game()
{
    /* Wait for color selection */
    enum strat_color_t color = wait_for_color_selection();

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot\n");
    strategy_auto_position(MIRROR_X(color, 600), 200, 90, color);
    NOTICE("Robot positioned at x: 600[mm], y: 200[mm], a: 90[deg]\n");

    /* Wait for starter to begin */
    wait_for_starter();
    NOTICE("Starting game\n");

    while (true) {
        /* Go to lunar module */
        strategy_goto_avoid_retry(MIRROR_X(color, 780), 1340, MIRROR_A(color, 45), TRAJ_FLAGS_ALL, -1);

        /* Push lunar module */
        trajectory_d_rel(&robot.traj, 100.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);
        trajectory_d_rel(&robot.traj, -100.);
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

        /* Go back to home */
        strategy_goto_avoid_retry(MIRROR_X(color, 600), 200, MIRROR_A(color, 90), TRAJ_FLAGS_ALL, -1);

        DEBUG("Game ended!\nInsert coin to play more.\n");
        chThdSleepSeconds(1);

        wait_for_starter();
    }
}

void strategy_play_game(void *p)
{
    (void) p;
    chRegSetThreadName("strategy");

    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));
    NOTICE("Strategy is ready, waiting for autopositioning signal");

#ifdef DEBRA
    strategy_debra_play_game();
#else
    strategy_sandoi_play_game();
#endif
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, NULL);
}
