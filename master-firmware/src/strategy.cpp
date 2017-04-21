#include <ch.h>
#include <hal.h>

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
#include "arms/arms_controller.h"
#include "arms/hands_controller.h"
#include "can/rocket_driver.h"
#include "config.h"
#include "main.h"

#include "strategy.h"

int traj_end_flags;

static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
void strategy_play_game(void* robot);

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

void strategy_stop_robot(struct _robot* robot)
{
    trajectory_hardstop(&robot->traj);
    chThdSleepMilliseconds(200);
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
        map_update_opponent_obstacle(x_opp, y_opp, robot->opponent_size * 1.25, robot->robot_size);
    }

    /* Compute path */
    oa_reset();
    const point_t start = {
            position_get_x_float(&robot->pos),
            position_get_y_float(&robot->pos)
        };
    oa_start_end_points(start.x, start.y, x_mm, y_mm);
    oa_process();

    /* Retrieve path */
    point_t *points;
    int num_points = oa_get_path(&points);
    DEBUG("Path to (%d, %d) computed with %d points", x_mm, y_mm, num_points);
    if (num_points <= 0) {
        WARNING("No path found!");
        strategy_stop_robot(robot);
        return false;
    }

    /* Execute path, one waypoint at a time */
    int end_reason = 0;

    for (int i = 0; i < num_points; i++) {
        DEBUG("Going to x: %.1fmm y: %.1fmm", points[i].x, points[i].y);

        trajectory_goto_forward_xy_abs(&robot->traj, points[i].x, points[i].y);
        end_reason = trajectory_wait_for_end(robot, &bus, traj_end_flags);

        if (end_reason != TRAJ_END_GOAL_REACHED) {
            break;
        }
    }

    if (end_reason == TRAJ_END_GOAL_REACHED) {
        trajectory_a_abs(&robot->traj, a_deg);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

        DEBUG("Goal reached successfully");

        return true;
    } else if (end_reason == TRAJ_END_OPPONENT_NEAR) {
        strategy_stop_robot(robot);
        WARNING("Stopping robot because opponent too close");
    } else if (end_reason == TRAJ_END_COLLISION) {
        strategy_stop_robot(robot);
        WARNING("Stopping robot because collision detected");
    } else if (end_reason == TRAJ_END_TIMER) {
        strategy_stop_robot(robot);
        WARNING("Stopping robot because game has ended !");
    } else {
        WARNING("Trajectory ended with reason %d", end_reason);
    }

    return false;
}

bool strategy_goto_avoid_retry(struct _robot* robot, int x_mm, int y_mm, int a_deg, int num_retries)
{
    bool finished = false;
    int counter = 0;

    while (!finished) {
        DEBUG("Try #%d", counter);
        finished = strategy_goto_avoid(robot, x_mm, y_mm, a_deg);
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
    struct _robot *robot{nullptr};
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

        const char* motor_names[6] = {"left-shoulder", "left-elbow", "left-wrist", "right-shoulder", "right-elbow", "right-wrist"};
        int motor_dirs[6] = {1, 1, 1, -1, -1, -1};
        float motor_speeds[6] = {0.8, 0.8, 4.0, 0.8, 0.8, 4.0};
        float motor_indexes[6];
        arms_auto_index(motor_names, motor_dirs, motor_speeds, 6, motor_indexes);

        arms_set_motor_index(left_arm.z_args, 0);
        arms_set_motor_index(left_arm.shoulder_args, motor_indexes[0]);
        arms_set_motor_index(left_arm.elbow_args, motor_indexes[1]);
        arms_set_motor_index(left_hand.wrist_args, motor_indexes[2]);

        arms_set_motor_index(right_arm.z_args, 0);
        arms_set_motor_index(right_arm.shoulder_args, motor_indexes[3]);
        arms_set_motor_index(right_arm.elbow_args, motor_indexes[4]);
        arms_set_motor_index(right_hand.wrist_args, motor_indexes[5]);

        left_hand.enable_control = true;
        right_hand.enable_control = true;

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
        scara_goto(&left_arm, -150, 70, 20, COORDINATE_ROBOT, 1.);
        scara_goto(&right_arm, 150, -70, 20, COORDINATE_ROBOT, 1.);
        chThdSleepSeconds(1.);
        state.arms_are_deployed = false;
        return true;
    }
};

struct GotoLocation : public goap::Action<DebraState> {
    enum Location m_loc;
    int m_x_mm, m_y_mm, m_a_deg;

    GotoLocation(enum Location loc, int x_mm, int y_mm, int a_deg) :
        m_loc(loc), m_x_mm(x_mm), m_y_mm(y_mm), m_a_deg(a_deg)
    {
    }

    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed && !state.has_moved && state.location_accessible[(int)m_loc];
    }

    DebraState plan_effects(DebraState state)
    {
        state.near_location = m_loc;
        state.has_moved = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Goto location %d: %dmm %dmm %ddeg", (int)m_loc, m_x_mm, m_y_mm, m_a_deg);
        state.near_location = Other;
        if (strategy_goto_avoid(state.robot, m_x_mm, m_y_mm, m_a_deg)) {
            state.near_location = m_loc;
            state.has_moved = true;
            return true;
        } else {
            state.location_accessible[(int)m_loc] = false;
            return false;
        }
    }
};

struct PickCylinder : public goap::Action<DebraState> {
    enum Location m_loc;
    int m_x_mm, m_y_mm;
    int m_cylinder_index;

    PickCylinder(enum Location loc, int x_mm, int y_mm, int idx) :
        m_loc(loc), m_x_mm(x_mm), m_y_mm(y_mm), m_cylinder_index(idx)
    {
    }

    bool can_run(DebraState state)
    {
        return state.near_location == m_loc && state.cylinder_present[m_cylinder_index];
    }

    DebraState plan_effects(DebraState state)
    {
        state.cylinder_count++;
        state.arms_are_deployed = true;
        state.has_moved = false;
        state.cylinder_present[m_cylinder_index] = false;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Pick cylinder %d", (int)m_loc);
        scara_goto(&left_arm, m_x_mm, m_y_mm, 20, COORDINATE_TABLE, 5.);
        chThdSleepSeconds(5);

        state.cylinder_count++;
        state.arms_are_deployed = true;
        state.has_moved = false;
        state.cylinder_present[m_cylinder_index] = false;
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
        return (state.cylinder_count == 2) && !state.arms_are_deployed;
    }
};

void strategy_debra_play_game(struct _robot* robot)
{
    /* Wait for color selection */
    enum strat_color_t color = wait_for_color_selection();

    /* Disable obstacle avoidance / game time */
    traj_end_flags = TRAJ_END_GOAL_REACHED;

    int len;
    InitGoal init_goal;
    IndexArms index_arms;
    RetractArms retract_arms;
    GotoLocation goto_region[] = {
        GotoLocation(Cylinder0,  900,  200,   0),
        GotoLocation(Cylinder1, 1200,  500,  90),
        GotoLocation(Cylinder2,  400,  700, 180),
        GotoLocation(Cylinder3,  700, 1100,  90),
        GotoLocation(Cylinder4, 1050, 1180,  45),
        GotoLocation(Cylinder5,  600, 1600,  45),
    };
    PickCylinder pick_cylinder[] = {
        PickCylinder(Cylinder0,  950,  200, 0), // When starting in this region, it's removed
        PickCylinder(Cylinder1, 1000,  600, 1),
        PickCylinder(Cylinder2,  200,  600, 2),
        PickCylinder(Cylinder3,  500, 1100, 3),
        PickCylinder(Cylinder4,  900, 1400, 4),
        PickCylinder(Cylinder5,  800, 1850, 5),
    };

    DebraState state;
    state.robot = robot;

    const int max_path_len = 10;
    goap::Action<DebraState> *path[max_path_len] = {nullptr};

    goap::Action<DebraState> *actions[] = {
        &index_arms,
        &retract_arms,
        &goto_region[0],
        &goto_region[1],
        &goto_region[2],
        &goto_region[3],
        &goto_region[4],
        &goto_region[5],
        &pick_cylinder[1],
        &pick_cylinder[2],
        &pick_cylinder[3],
        &pick_cylinder[4],
        &pick_cylinder[5],
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
    strategy_auto_position(300, 200, -90, robot->robot_size, color, robot, &bus);
    position_set(&robot->pos, MIRROR_X(color, 300), 200 + 382, -90);

    // Second alignement only in y at starting area
    strategy_goto_avoid_retry(robot, 900, 200, -90, -1);
    strategy_align_y(200, robot->robot_size, robot, &bus);
    trajectory_a_abs(&robot->traj, 90);
    trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot->pos), position_get_y_s16(&robot->pos), position_get_x_s16(&robot->pos));

    /* Wait for starter to begin */
    wait_for_starter();
    traj_end_flags = TRAJ_FLAGS_STD;
    trajectory_game_timer_reset(robot);
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

void strategy_sandoi_play_game(struct _robot* robot)
{
    /* Wait for color selection */
    enum strat_color_t color = wait_for_color_selection();

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot\n");
    strategy_auto_position(600, 200, 90, robot->robot_size, color, robot, &bus);
    NOTICE("Robot positioned at x: 600[mm], y: 200[mm], a: 90[deg]\n");

    /* Wait for starter to begin */
    wait_for_starter();
    NOTICE("Starting game\n");

    while (true) {
        /* Go to lunar module */
        strategy_goto_avoid_retry(robot, 780, 1340, 45, -1);

        /* Push lunar module */
        trajectory_d_rel(&robot->traj, 100.);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);
        trajectory_d_rel(&robot->traj, -100.);
        trajectory_wait_for_end(robot, &bus, TRAJ_END_GOAL_REACHED);

        /* Go back to home */
        strategy_goto_avoid_retry(robot, 600, 200, 90, -1);

        DEBUG("Game ended!\nInsert coin to play more.\n");
        chThdSleepSeconds(1);

        wait_for_starter();
    }
}

void strategy_play_game(void* _robot)
{
    chRegSetThreadName("strategy");

    struct _robot* robot = (struct _robot*)_robot;

    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));
    NOTICE("Strategy is ready, waiting for autopositioning signal");

#ifdef DEBRA
    strategy_debra_play_game(robot);
#else
    strategy_sandoi_play_game(robot);
#endif
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
