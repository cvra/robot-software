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
#include "base/base_helpers.h"
#include "base/map.h"
#include "scara/scara_trajectories.h"
#include "arms/arms_controller.h"
#include "arms/arm_trajectory_manager.h"
#include "arms/arm.h"
#include "lever/lever_module.h"
#include "config.h"
#include "control_panel.h"
#include "main.h"

#include "strategy.h"
#include "strategy/actions.h"
#include "strategy/goals.h"
#include "strategy/state.h"


static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void strategy_wait_ms(int ms);

void strategy_play_game(void);

#define MIRROR_RIGHT_LEVER(color) (color == YELLOW ? (&right_lever) : (&left_lever))
#define MIRROR_LEFT_LEVER(color) (color == YELLOW ? (&left_lever) : (&right_lever))

#define BUTTON_IS_PRESSED(in) (control_panel_read(in) == true) // Active high

static enum strat_color_t wait_for_color_selection(void)
{
    strat_color_t color = YELLOW;

    while (!BUTTON_IS_PRESSED(BUTTON_YELLOW) && !BUTTON_IS_PRESSED(BUTTON_GREEN)) {
        control_panel_set(LED_YELLOW);
        control_panel_set(LED_GREEN);
        strategy_wait_ms(100);

        control_panel_clear(LED_YELLOW);
        control_panel_clear(LED_GREEN);
        strategy_wait_ms(100);
    }

    if (BUTTON_IS_PRESSED(BUTTON_GREEN)) {
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
    /* Wait for a rising edge */
    while (control_panel_read(STARTER)) {
        strategy_wait_ms(10);
    }
    while (!control_panel_read(STARTER)) {
        strategy_wait_ms(10);
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
    trajectory_hardstop(&robot.traj);
    strategy_wait_ms(200);
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
        control_panel_set(LED_PC);
        strategy_stop_robot();
        strategy_wait_ms(100);
        control_panel_clear(LED_PC);
        strategy_stop_robot();
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

void strategy_score_init()
{
    static messagebus_topic_t topic;
    static MUTEX_DECL(topic_lock);
    static CONDVAR_DECL(topic_condvar);
    static int score;

    messagebus_topic_init(&topic, &topic_lock, &topic_condvar, &score, sizeof(score));
    messagebus_advertise_topic(&bus, &topic, "/encoders");
}

void strategy_score_increase(int increment)
{
    static int score = 0;
    score += increment;
    messagebus_topic_t *score_topic = messagebus_find_topic_blocking(&bus, "/score");
    messagebus_topic_publish(score_topic, &score, sizeof(score));
}

struct IndexArms : actions::IndexArms {
    bool execute(RobotState &state)
    {
        NOTICE("Indexing arms!");

        /* Z axis indexing */
        cvra_arm_motor_t* z_motors[] = {
            (cvra_arm_motor_t *)main_arm.hw_interface.z_joint.args,
        };
        float z_speeds[] = {-20};
        arms_auto_index(z_motors, z_speeds, sizeof(z_speeds) / sizeof(float));

        z_motors[0]->index += config_get_scalar("master/arms/motor_offsets/z-joint");

        /* Arm indexing */
        cvra_arm_motor_t* motors[] = {
            (cvra_arm_motor_t *)main_arm.hw_interface.shoulder_joint.args,
            (cvra_arm_motor_t *)main_arm.hw_interface.elbow_joint.args,
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

    bool execute(RobotState &state)
    {
        NOTICE("Retracting arms!");
        state.arms_are_deployed = false;

        scara_control_mode_joint(&main_arm);

        if (m_color == YELLOW) {
            main_arm.shoulder_mode = SHOULDER_BACK;
        } else {
            main_arm.shoulder_mode = SHOULDER_FRONT;
        }
        scara_goto(&main_arm, {170., 0., 295.}, COORDINATE_ARM, {300, 300, 300});
        strategy_wait_ms(500);
        scara_control_mode_cartesian(&main_arm);

        scara_goto(&main_arm, {-20., MIRROR(m_color, 120.), 295.}, COORDINATE_ROBOT, {300, 300, 300});
        arm_traj_wait_for_end();

        return true;
    }
};

void strat_scara_goto_blocking(position_3d_t pos, scara_coordinate_t system, velocity_3d_t max_vel)
{
    scara_goto(&main_arm, pos, system, max_vel);
    arm_traj_wait_for_end();
}

void strat_scara_push_x(float dx, scara_coordinate_t system, velocity_3d_t max_vel)
{
    const position_3d_t last_pos = scara_position(&main_arm, system);
    scara_goto(&main_arm, {last_pos.x + dx, last_pos.y, last_pos.z}, system, max_vel);
    arm_traj_wait_for_event(ARM_READY | ARM_BLOCKED_XY);
}

void strat_scara_push_y(float dy, scara_coordinate_t system, velocity_3d_t max_vel)
{
    const position_3d_t last_pos = scara_position(&main_arm, system);
    scara_goto(&main_arm, {last_pos.x, last_pos.y + dy, last_pos.z}, system, max_vel);
    arm_traj_wait_for_event(ARM_READY | ARM_BLOCKED_XY);
}

void strat_pick_cube(point_t xy, float z_start)
{
    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_ARM);
    strat_scara_goto_blocking({200, 0, last_pos.z}, COORDINATE_ARM, {300, 300, 300});
    strat_scara_goto_blocking({xy.x, xy.y, z_start}, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_goto_blocking({xy.x, xy.y, 60}, COORDINATE_TABLE, {300, 300, 300});

    hand_set_pump(&main_hand, PUMP_ON);
    strategy_wait_ms(200.);

    strat_scara_goto_blocking({xy.x, xy.y, z_start}, COORDINATE_TABLE, {300, 300, 300});
}

void strat_deposit_cube(float x, float y, int num_cubes_in_tower)
{
    const float z = (num_cubes_in_tower + 1) * 70.;
    const float margin_z = 20;

    scara_hold_position(&main_arm, COORDINATE_ARM);
    arm_traj_wait_for_end();

    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_ARM);
    strat_scara_goto_blocking({200, 0, last_pos.z}, COORDINATE_ARM, {300, 300, 300});
    strat_scara_goto_blocking({x, y, z + margin_z}, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_goto_blocking({x, y, z}, COORDINATE_TABLE, {300, 300, 300});

    hand_set_pump(&main_hand, PUMP_REVERSE);
    strat_scara_goto_blocking({x, y, z + margin_z}, COORDINATE_TABLE, {300, 300, 300});
    hand_set_pump(&main_hand, PUMP_OFF);
}

struct BuildTower : actions::BuildTower {
    enum strat_color_t m_color;

    BuildTower(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState &state)
    {
        NOTICE("Building tower!");
        state.arms_are_deployed = true;

        lever_t* lever = MIRROR_LEFT_LEVER(m_color);

        strategy_goto_avoid_retry(MIRROR_X(m_color, 500), 300, MIRROR_A(m_color, -135), TRAJ_FLAGS_ALL, -1);

        lever_deploy(lever);
        strategy_wait_ms(500);

        se2_t blocks_pose = lever_deposit(lever, base_get_robot_pose(&robot.pos));
        strategy_wait_ms(500);

        lever_push_and_retract(lever);
        strategy_wait_ms(500);

        strategy_goto_avoid_retry(MIRROR_X(m_color, 500), 300, MIRROR_A(m_color, -225), TRAJ_FLAGS_ALL, -1);

        strat_pick_cube(strategy_block_pos(blocks_pose, BLOCK_BLACK), 200);
        strat_deposit_cube(MIRROR_X(m_color, 470), 90, 0);
        strategy_score_increase(1);

        strat_pick_cube(strategy_block_pos(blocks_pose, BLOCK_GREEN), 200);
        strat_deposit_cube(MIRROR_X(m_color, 470), 90, 1);
        strategy_score_increase(2);

        strat_pick_cube(strategy_block_pos(blocks_pose, BLOCK_YELLOW), 200);
        strat_deposit_cube(MIRROR_X(m_color, 470), 90, 2);
        strategy_score_increase(3);

        strat_pick_cube(strategy_block_pos(blocks_pose, BLOCK_RED), 200);
        strat_deposit_cube(MIRROR_X(m_color, 470), 90, 3);
        strategy_score_increase(4);

        state.tower_built = true;

        return true;
    }
};

struct PickupBlocks : actions::PickupBlocks {
    enum strat_color_t m_color;

    PickupBlocks(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState &state)
    {
        NOTICE("Picking up some blocks");

        lever_t* lever = MIRROR_LEFT_LEVER(m_color);

        se2_t blocks_pose = se2_create_xya(MIRROR_X(m_color, 850), 540, 0);

        strategy_goto_avoid_retry(MIRROR_X(m_color, 690), 380, MIRROR_A(m_color, -45), TRAJ_FLAGS_ALL, -1);

        lever_deploy(lever);
        strategy_wait_ms(500);

        lever_pickup(lever, base_get_robot_pose(&robot.pos), blocks_pose);
        strategy_wait_ms(1000);

        lever_retract(lever);
        strategy_wait_ms(500);

        state.has_blocks = true;

        return true;
    }
};

void strat_push_switch_on(float x, float y, float z, float y_push)
{
    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_TABLE);
    strat_scara_goto_blocking({x,      y, last_pos.z}, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_goto_blocking({x,      y,          z}, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_push_y(                        y_push, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_goto_blocking({x,      y,          z}, COORDINATE_TABLE, {300, 300, 300});
    strat_scara_goto_blocking({x,      y, last_pos.z}, COORDINATE_TABLE, {300, 300, 300});
}

struct TurnSwitchOn : public actions::TurnSwitchOn {
    enum strat_color_t m_color;

    TurnSwitchOn(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool execute(RobotState& state)
    {
        NOTICE("Turning switch on");

        state.arms_are_deployed = true;
        state.blocks_on_map = false;
        if (!strategy_goto_avoid_retry(MIRROR_X(m_color, 1130), 250, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL, -1)) {
            return false;
        }

        strat_push_switch_on(MIRROR_X(m_color, 1130), 50, 120, -120);
        strategy_score_increase(25);

        state.switch_on = true;
        return true;
    }
};

void strategy_debra_play_game(void)
{
    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));
    strategy_score_init(); // Initialize score to 0

    NOTICE("Waiting for color selection...");
    enum strat_color_t color = wait_for_color_selection();

    int len;
    RobotState state;

    InitGoal init_goal;
    GameGoal game_goal;

    IndexArms index_arms;
    RetractArms retract_arms(color);
    BuildTower build_tower(color);
    PickupBlocks pickup_blocks(color);
    TurnSwitchOn turn_switch_on(color);

    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};

    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &build_tower,
        &pickup_blocks,
        &turn_switch_on,
    };

    static goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    lever_retract(&right_lever);
    lever_retract(&left_lever);

    NOTICE("Getting arms ready...");
    len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(color, 200), 200, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Init score */
    strategy_score_increase(5); // Domotic panel

    /* Wait for starter to begin */
    wait_for_starter();
    trajectory_game_timer_reset();
    strategy_score_increase(10);

    NOTICE("Starting game...");
    len = planner.plan(state, game_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    while (true) {
        NOTICE("Game ended!");
        strategy_wait_ms(1000);
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
        strategy_wait_ms(1000);

        wait_for_starter();
    }
}

void strategy_play_game(void *p)
{
    (void) p;
    chRegSetThreadName("strategy");

    /* Initialize map and path planner */
    map_init(config_get_integer("master/robot_size_x_mm"));

    /* Prepare score publisher */
    static messagebus_topic_t score_topic;
    static MUTEX_DECL(score_lock);
    static CONDVAR_DECL(score_condvar);
    static int score_value;
    messagebus_topic_init(&score_topic, &score_lock,
                          &score_condvar, &score_value, sizeof(int));
    messagebus_advertise_topic(&bus, &score_topic, "/score");

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
