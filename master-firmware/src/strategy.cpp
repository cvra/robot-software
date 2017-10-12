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
#include "control_panel.h"
#include "main.h"

#include "strategy.h"


static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void strategy_wait_ms(int ms);

void strategy_play_game(void);

static scara_t* mirror_left_arm(enum strat_color_t color);
static scara_t* mirror_right_arm(enum strat_color_t color);
static hand_t* mirror_left_hand(enum strat_color_t color);
static hand_t* mirror_right_hand(enum strat_color_t color);

#define BUTTON_IS_PRESSED(in) (control_panel_read(in) == false) // Active low
#define ARM_TRAJ_SYNCHRONOUS(arm, traj) do { \
        strategy_wait_ms(strategy_set_arm_trajectory(arm, traj, \
                         sizeof(traj) / sizeof(arm_waypoint_t))); \
    } while (0)

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


static scara_t* mirror_left_arm(enum strat_color_t color)
{
    return color == YELLOW ? &left_arm : &right_arm;
}
static scara_t* mirror_right_arm(enum strat_color_t color)
{
    return color == YELLOW ? &right_arm : &left_arm;
}

static hand_t* mirror_left_hand(enum strat_color_t color)
{
    return color == YELLOW ? &left_hand : &right_hand;
}
static hand_t* mirror_right_hand(enum strat_color_t color)
{
    return color == YELLOW ? &right_hand : &left_hand;
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
    int score{0};
    bool arms_are_indexed{false};
    bool arms_are_deployed{true};
    int cylinder_count{0};
    bool construction_area_free{false};
    bool cylinder_taken[5] = {false}; // TODO how many cylinders
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

        /* Z axis indexing */
        cvra_arm_motor_t* z_motors[] = {
            (cvra_arm_motor_t *)left_arm.z_args,
            (cvra_arm_motor_t *)right_arm.z_args,
        };
        float z_speeds[] = {-20, -20};
        arms_auto_index(z_motors, z_speeds, sizeof(z_speeds) / sizeof(float));

        z_motors[0]->index += config_get_scalar("master/arms/motor_offsets/left-z");
        z_motors[1]->index += config_get_scalar("master/arms/motor_offsets/right-z");

        /* Arm indexing */
        cvra_arm_motor_t* motors[] = {
            (cvra_arm_motor_t *)left_arm.shoulder_args,
            (cvra_arm_motor_t *)left_arm.elbow_args,
            (cvra_arm_motor_t *)right_arm.shoulder_args,
            (cvra_arm_motor_t *)right_arm.elbow_args,
        };
        float motor_speeds[] = {0.8, 0.8, -0.8, -0.8};
        arms_auto_index(motors, motor_speeds, sizeof(motor_speeds) / sizeof(float));

        motors[0]->index += config_get_scalar("master/arms/motor_offsets/left-shoulder");
        motors[1]->index += config_get_scalar("master/arms/motor_offsets/left-elbow");
        motors[2]->index += config_get_scalar("master/arms/motor_offsets/right-shoulder");
        motors[3]->index += config_get_scalar("master/arms/motor_offsets/right-elbow");

        state.arms_are_indexed = true;
        return true;
    }
};

struct RetractArms : public goap::Action<DebraState> {
    enum strat_color_t m_color;

    RetractArms(enum strat_color_t color)
        : m_color(color)
    {
    }
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

        // Enable joint position control
        scara_ugly_mode_enable(&left_arm);
        scara_ugly_mode_enable(&right_arm);

        // First move arms up

        scara_move_z(&left_arm, 120, COORDINATE_ROBOT, 0.5);
        scara_move_z(&right_arm, 120, COORDINATE_ROBOT, 0.5);
        strategy_wait_ms(1000);

        scara_goto(&left_arm, -150, 120,  120, RADIANS(180), COORDINATE_ROBOT, 1.);
        scara_goto(&right_arm, -150, -120, 120, RADIANS(180), COORDINATE_ROBOT, 1.);
        strategy_wait_ms(1000);

        scara_goto(&left_arm, -150, 80,  120, RADIANS(180), COORDINATE_ROBOT, 1.);
        scara_goto(&right_arm, -150, -80, 120, RADIANS(180), COORDINATE_ROBOT, 1.);
        strategy_wait_ms(1000);

        state.arms_are_deployed = false;
        return true;
    }
};

struct PushMultiColoredCylinder : public goap::Action<DebraState> {
    enum strat_color_t m_color;

    PushMultiColoredCylinder(enum strat_color_t color)
        : m_color(color)
    {
    }

    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed && (state.cylinder_count > 1) && !state.construction_area_free;
    }

    DebraState plan_effects(DebraState state)
    {
        state.score += 10;
        state.arms_are_deployed = true;
        state.construction_area_free = true;
        return state;
    }

    bool execute(DebraState &state)
    {
        scara_t* arm = mirror_left_arm(m_color);


        // Approach with wheelbase
        if (!strategy_goto_avoid(MIRROR_X(m_color, 1040), 1270, MIRROR_A(m_color, 45), TRAJ_FLAGS_ALL)) {
            return false;
        }

        scara_ugly_mode_enable(arm);

        // Position arm
        int x=152, y = (m_color == YELLOW ? 190 : -190), z=120;
        scara_goto(arm, x, y, z, RADIANS(0), COORDINATE_ROBOT, 2.);
        strategy_wait_ms(2000);

        scara_move_z(arm, 150, COORDINATE_ROBOT, 1.);
        strategy_wait_ms(1000);


        state.score += 10;
        state.arms_are_deployed = true;
        state.construction_area_free = true;

        return true;

    }
};

struct CollectCylinder : public goap::Action<DebraState> {
    int m_index;
    enum strat_color_t m_color;
    float x_mm, y_mm;

    CollectCylinder(unsigned index, enum strat_color_t color, float x_cylinder_mm, float y_cylinder_mm)
        : m_index(index), m_color(color), x_mm(x_cylinder_mm), y_mm(y_cylinder_mm)
    {
    }

    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed && state.cylinder_count < 4 && !state.cylinder_taken[m_index];
    }

    DebraState plan_effects(DebraState state)
    {
        state.cylinder_count ++;
        state.arms_are_deployed = true;
        state.cylinder_taken[m_index] = true;
        return state;
    }

    bool execute(DebraState &state)
    {

        scara_t* arm = mirror_left_arm(m_color);
        hand_t* hand = mirror_left_hand(m_color);

        int slot = state.cylinder_count;
        NOTICE("Collecting cylinder %d using slot %d", m_index, slot);

        // Approach rocket with wheelbase
        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm + 200), y_mm - 100, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            state.arms_are_deployed = true;
            return false;
        }

        scara_ugly_mode_enable(arm);

        // Go above cylinder
        scara_move_z(arm, 120, COORDINATE_ROBOT, 0.5);

        // Prepare arm
        scara_goto(arm, MIRROR_X(m_color, x_mm), y_mm - 110, 50, RADIANS(MIRROR_A(m_color, 90)), COORDINATE_TABLE, 1.);
        strategy_wait_ms(2000);

        // Select tool
        hand_set_finger(hand, slot, FINGER_OPEN);
        strategy_wait_ms(500);

        // Approach cylinder
        scara_ugly_mode_disable(arm);
        arm_waypoint_t pick_cylinder_traj[] = {
            {.x=MIRROR_X(m_color, x_mm), .y=y_mm, .z=50, .a=MIRROR_A(m_color, 90), .coord=COORDINATE_TABLE, .dt=0, .l3=160},
            {.x=MIRROR_X(m_color, x_mm), .y=y_mm, .z=50, .a=MIRROR_A(m_color, 90), .coord=COORDINATE_TABLE, .dt=1000, .l3=55},
        };
        ARM_TRAJ_SYNCHRONOUS(arm, pick_cylinder_traj);

        // Get cylinder
        hand_set_finger(hand, slot, FINGER_CLOSED);
        strategy_wait_ms(500);

        scara_ugly_mode_enable(arm);

        state.cylinder_count ++;
        state.arms_are_deployed = true;
        state.cylinder_taken[m_index] = true;
        return true;
    }
};

struct DepositCylinder : public goap::Action<DebraState> {
    enum strat_color_t m_color;
    int m_drop_count;

    DepositCylinder(enum strat_color_t color)
        : m_color(color), m_drop_count(0)
    {
    }

    bool can_run(DebraState state)
    {
        return !state.arms_are_deployed && (state.cylinder_count > 0) && state.construction_area_free;
    }

    DebraState plan_effects(DebraState state)
    {
        state.score += 10;
        state.arms_are_deployed = true;
        state.cylinder_count --;
        return state;
    }

    bool execute(DebraState &state)
    {
        scara_t* arm = mirror_left_arm(m_color);
        hand_t* hand = mirror_left_hand(m_color);

        state.arms_are_deployed = true;

        while (state.cylinder_count > 0) {
            // Select slot
            int slot = state.cylinder_count - 1;
            NOTICE("Depositing cylinder in slot %d", slot);

            // Approach with wheelbase
            if (!strategy_goto_avoid(MIRROR_X(m_color, 250), 800 + 120 * m_drop_count, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
                return false;
            }

            // Prepare arm
            int x = 43, y = (m_color == YELLOW ? 206 : -206), z = 160, a = 180 - 90 * slot;

            scara_move_z(arm, z, COORDINATE_ROBOT, 1.);
            strategy_wait_ms(1000);

            scara_goto_with_length(arm, x, y, z, RADIANS(a), COORDINATE_ROBOT, 4., 0);
            strategy_wait_ms(4000);

            // Drop cylinder
            hand_set_finger(hand, slot, FINGER_OPEN);
            strategy_wait_ms(1000);

            hand_set_finger(hand, slot, FINGER_CLOSED);
            strategy_wait_ms(500);

            m_drop_count ++;
            state.score += 10;
            state.arms_are_deployed = true;
            state.cylinder_count --;
        }

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
        return state.cylinder_taken[0] && state.cylinder_taken[1] &&
            state.cylinder_count == 0 &&
            !state.arms_are_deployed;
    }
};

void strategy_debra_play_game(void)
{
    /* Wait for color selection */
    enum strat_color_t color = wait_for_color_selection();

    int len;
    InitGoal init_goal;
    GameGoal game_goal;
    IndexArms index_arms;
    RetractArms retract_arms(color);
    PushMultiColoredCylinder push_multi_cylinder(color);

    CollectCylinder collect_cylinder_1(0, color, 1000, 600);
    CollectCylinder collect_cylinder_2(1, color, 500, 1100);

    DepositCylinder deposit_cylinder(color);

    DebraState state;

    const int max_path_len = 10;
    goap::Action<DebraState> *path[max_path_len] = {nullptr};

    goap::Action<DebraState> *actions[] = {
        &index_arms,
        &retract_arms,
        &push_multi_cylinder,
        &collect_cylinder_1,
        &collect_cylinder_2,
        &deposit_cylinder,
    };

    goap::Planner<DebraState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    for (size_t i = 0; i < 4; i++) {
        hand_set_finger(&right_hand, i, FINGER_RETRACTED);
    }

    NOTICE("Getting arms ready...");
    len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;

    // First alignment
    strategy_auto_position(MIRROR_X(color, 300), 200, MIRROR_A(color, -90), color);
    robot.pos.pos_d.y += 382;
    robot.pos.pos_s16.y += 382;

    // Second alignement only in y at starting area
    robot.base_speed = BASE_SPEED_FAST;
    strategy_goto_avoid_retry(MIRROR_X(color, 890), 200, MIRROR_A(color, -90), TRAJ_END_GOAL_REACHED, -1);
    robot.base_speed = BASE_SPEED_INIT;

    strategy_align_y(170);
    trajectory_a_abs(&robot.traj, MIRROR_A(color, 90));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();
    trajectory_game_timer_reset();
    rocket_program_launch_time(GAME_DURATION + 1);

    NOTICE("Starting game");
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
            strategy_wait_ms(1000);
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
