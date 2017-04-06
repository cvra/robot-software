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
#include "config.h"
#include "main.h"

#include "strategy.h"


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
            position_get_x_float(&robot->pos),
            position_get_y_float(&robot->pos)
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

struct DebraState {
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    bool lunar_module_down{false};
    bool is_near_lunar_module{false};
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

        arms_set_motor_index(left_arm.shoulder_args, motor_indexes[0]);
        arms_set_motor_index(left_arm.elbow_args, motor_indexes[1]);
        arms_set_motor_index(right_arm.shoulder_args, motor_indexes[3]);
        arms_set_motor_index(right_arm.elbow_args, motor_indexes[4]);

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
        strategy_arm_goto(state.robot, &left_arm, -150, 70, 0, COORDINATE_ROBOT, 1.);
        strategy_arm_goto(state.robot, &right_arm, 150, -70, 0, COORDINATE_ROBOT, 1.);
        chThdSleepSeconds(1.);
        state.arms_are_deployed = false;
        return true;
    }
};

struct GotoLunarModule : public goap::Action<DebraState> {
    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed;
    }

    DebraState plan_effects(DebraState state)
    {
        state.is_near_lunar_module = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Goto lunar module");
        if (strategy_goto_avoid(state.robot, 1050, 1180, 45)) {
            state.is_near_lunar_module = true;
        }
        return state.is_near_lunar_module;
    }
};

struct PushLunarModule : public goap::Action<DebraState> {
    bool can_run(DebraState state)
    {
        return state.is_near_lunar_module;
    }

    DebraState plan_effects(DebraState state)
    {
        state.lunar_module_down = true;
        state.arms_are_deployed = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        NOTICE("Push lunar module");
        strategy_arm_goto(state.robot, &left_arm, 960, 1460, 0, COORDINATE_TABLE, 5.);
        chThdSleepSeconds(5);

        state.lunar_module_down = true;
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
        return state.lunar_module_down && !state.arms_are_deployed;
    }
};

void strategy_debra_play_game(struct _robot* robot, enum strat_color_t color)
{
    (void) robot;
    (void) color;
    int len;

    InitGoal init_goal;
    IndexArms index_arms;
    RetractArms retract_arms;
    GotoLunarModule goto_lunar_module;
    PushLunarModule push_lunar_module;

    DebraState state;
    state.robot = robot;

    const int max_path_len = 10;
    goap::Action<DebraState> *path[max_path_len] = {nullptr};

    goap::Action<DebraState> *actions[] = {
        &index_arms,
        &retract_arms,
        &goto_lunar_module,
        &push_lunar_module,
    };

    goap::Planner<DebraState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    GameGoal game_goal;
    len = planner.plan(state, game_goal, path, max_path_len);
    NOTICE("Plan length: %d", len);

    wait_for_autoposition_signal();
    NOTICE("Getting arms ready...");
    len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");
    strategy_auto_position(600, 200, 90, robot->robot_size, color, robot, &bus);
    NOTICE("Robot positioned at x: 600[mm], y: 200[mm], a: 90[deg]");


    /* Wait for starter to begin */
    wait_for_starter();
    NOTICE("Starting game");
    len = planner.plan(state, game_goal, path, max_path_len);
    NOTICE("Plan length: %d", len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }
}

void strategy_sandoi_play_game(struct _robot* robot, enum strat_color_t color)
{
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
    enum strat_color_t color = YELLOW;

    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));
    NOTICE("Strategy is ready, waiting for autopositioning signal");

#ifdef DEBRA
    strategy_debra_play_game(robot, color);
#else
    strategy_sandoi_play_game(robot, color);
#endif
}

void strategy_start(void)
{
    static THD_WORKING_AREA(strategy_thd_wa, 4096);
    chThdCreateStatic(strategy_thd_wa, sizeof(strategy_thd_wa), STRATEGY_PRIO, strategy_play_game, &robot);
}
