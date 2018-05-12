#include <ch.h>
#include <hal.h>
#include <array>

#include <error/error.h>
#include <blocking_detection_manager/blocking_detection_manager.h>
#include <trajectory_manager/trajectory_manager_utils.h>
#include <obstacle_avoidance/obstacle_avoidance.h>
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
#include "lever/lever_module.h"
#include "ballgun/ballgun_module.h"
#include "ballgun/ball_sense.h"
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


static enum strat_color_t wait_for_color_selection(void);
static void wait_for_autoposition_signal(void);
static void wait_for_starter(void);
static void strategy_wait_ms(int ms);

void strategy_play_game(void);

#define MIRROR_RIGHT_LEVER(color) (color == YELLOW ? (&right_lever) : (&left_lever))
#define MIRROR_LEFT_LEVER(color) (color == YELLOW ? (&left_lever) : (&right_lever))

static enum strat_color_t wait_for_color_selection(void)
{
    strat_color_t color = YELLOW;

    while (!control_panel_button_is_pressed(BUTTON_YELLOW) &&
           !control_panel_button_is_pressed(BUTTON_GREEN)) {
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
    trajectory_hardstop(&robot.traj);
    strategy_wait_ms(200);
}

bool strategy_goto_avoid(int x_mm, int y_mm, int a_deg, int traj_end_flags)
{
    /* Compute path */
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

static void update_panel_state_from_uwb(RobotState &state)
{
    messagebus_topic_t *topic;
    timestamp_t last_contact;
    topic = messagebus_find_topic(&bus, "/panel_contact_us");

    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    if (topic) {
        if (messagebus_topic_read(topic, &last_contact, sizeof(last_contact))) {
            auto delta = timestamp_duration_s(last_contact, timestamp_get());
            if (delta < 3.) {
                state.switch_on = true;
            } else {
                state.switch_on = false;
            }

            messagebus_topic_publish(state_topic, &state, sizeof(RobotState));
        }
    }
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

        main_arm.shoulder_mode = MIRROR_SHOULDER(m_color, SHOULDER_BACK);
        scara_goto(&main_arm, {170., 0., 295.}, COORDINATE_ARM, {300, 300, 300});
        strategy_wait_ms(500);

        scara_goto(&main_arm, {20., MIRROR(m_color, 90.), 295.}, COORDINATE_ROBOT, {300, 300, 300});
        strategy_wait_ms(500);

        scara_control_mode_cartesian(&main_arm);

        return true;
    }
};

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

bool strat_check_distance_to_hand_lower_than(float expected_value)
{
    bool success = true;

    float distance_to_tower;
    messagebus_topic_t* topic = messagebus_find_topic_blocking(&bus, "/hand_distance");

    if (messagebus_topic_read(topic, &distance_to_tower, sizeof(distance_to_tower))) {
        WARNING("Hand distance: %f", distance_to_tower);
        success = (distance_to_tower < expected_value);
    } else {
        WARNING("Hand distance sensor is not publishing");
    }

    return success;
}

bool strat_pick_cube(float x, float y)
{
    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_TABLE);
    strat_scara_goto_blocking({x, y, last_pos.z}, COORDINATE_TABLE, {300, 300, 300});

    strat_scara_goto({x, y, 80}, COORDINATE_TABLE, {300, 300, 300}, ARM_READY);
    strat_scara_goto({x, y, 60}, COORDINATE_TABLE, {200, 200, 200}, ARM_READY | ARM_BLOCKED_Z);

    bool cube_is_present = strat_check_distance_to_hand_lower_than(0.05f);

    if (cube_is_present) {
        hand_set_pump(&main_hand, PUMP_ON);
        strategy_wait_ms(500);
    }

    return cube_is_present;
}

float safe_z(float z)
{
    return fminf(z, 300.f);
}

bool strat_deposit_cube(float x, float y, int num_cubes_in_tower)
{
    const float z = (num_cubes_in_tower + 1) * 70.f;
    const float margin_z = 20.f;
    const float safe_z_with_margin = fminf(fmaxf(z + margin_z, 200.f), 300.f);

    scara_hold_position(&main_arm, COORDINATE_ARM);
    arm_traj_wait_for_end();

    arm_traj_manager_set_tolerances(&main_arm_traj_manager, 20.f, 20.f, 10.f);

    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_ARM);
    strat_scara_goto_blocking({last_pos.x, last_pos.y, safe_z_with_margin}, COORDINATE_ARM, {300, 300, 300});
    strat_scara_goto_blocking({x, y, safe_z_with_margin}, COORDINATE_TABLE, {150, 150, 150});
    strat_scara_goto_blocking({x, y, safe_z(z)}, COORDINATE_TABLE, {150, 150, 150});

    hand_set_pump(&main_hand, PUMP_REVERSE);
    strat_scara_goto_blocking({x, y, safe_z(z + margin_z)}, COORDINATE_TABLE, {300, 300, 300});
    hand_set_pump(&main_hand, PUMP_OFF);

    arm_traj_manager_set_tolerances(&main_arm_traj_manager, 10.f, 10.f, 2.f);

    return strat_check_distance_to_hand_lower_than(0.05f);
}

void strat_push_the_bee(point_t start, point_t end, float bee_height)
{
    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_ROBOT);

    scara_control_mode_joint(&main_arm);
    scara_goto(&main_arm, {start.x, start.y, last_pos.z}, COORDINATE_ROBOT, {300, 300, 300});
    strategy_wait_ms(2000);
    scara_goto(&main_arm, {start.x, start.y, bee_height}, COORDINATE_ROBOT, {300, 300, 300});
    strategy_wait_ms(2000);
    scara_goto(&main_arm, {end.x, end.y, bee_height}, COORDINATE_ROBOT, {300, 300, 300});
    strategy_wait_ms(2000);
    scara_control_mode_cartesian(&main_arm);

    strat_scara_goto_blocking({end.x, end.y, last_pos.z}, COORDINATE_ROBOT, {300, 300, 300});
}

void strat_push_the_bee_v2(point_t start, float bee_height, float forward_motion)
{
    const position_3d_t last_pos = scara_position(&main_arm, COORDINATE_ARM);

    strat_scara_goto_blocking({170.f, 0.f, last_pos.z}, COORDINATE_ARM, {300, 300, 300});
    strat_scara_goto_blocking({start.x, start.y, bee_height}, COORDINATE_ARM, {300, 300, 300});

    /* Push */
    trajectory_d_rel(&robot.traj, forward_motion);
    strategy_wait_ms(500);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    /* Go back */
    trajectory_d_rel(&robot.traj, -forward_motion);
    strategy_wait_ms(500);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);
}

bool strat_lever_is_full(enum lever_side_t lever_side)
{
    return true; // Temporarily disable this, investigating logic issues

    bool full;
    messagebus_topic_t *topic;

    if (lever_side == LEVER_SIDE_LEFT) {
        topic = messagebus_find_topic(&bus, "/lever/left");
    } else {
        topic = messagebus_find_topic(&bus, "/lever/right");
    }

    if (!topic) {
        WARNING("Could not find lever topic");
        return true;
    }

    if (!messagebus_topic_read(topic, &full, sizeof(full))) {
        WARNING("Lever topic was never published to.");
        return true;
    }

    return full;
}

struct PickupCubes : actions::PickupCubes {
    enum strat_color_t m_color;

    PickupCubes(enum strat_color_t color, int blocks_id)
        : actions::PickupCubes(blocks_id)
        , m_color(color)
    {
    }

    bool execute(RobotState &state)
    {
        const int x_mm = BLOCK_OF_CUBES_POS[blocks_id][0];
        const int y_mm = BLOCK_OF_CUBES_POS[blocks_id][1];
        NOTICE("Picking up some blocks at %d %d", MIRROR_X(m_color, x_mm), y_mm);

        enum lever_side_t lever_side = LEVER_SIDE_LEFT;
        lever_t* lever = MIRROR_LEFT_LEVER(m_color);
        int offset_a_deg = 0;

        if (state.lever_full_left) {
            lever_side = LEVER_SIDE_RIGHT;
            lever = MIRROR_RIGHT_LEVER(m_color);
            offset_a_deg = 180;
        }

        se2_t cubes_pose = se2_create_xya(MIRROR_X(m_color, x_mm), y_mm, 0);
        std::array<se2_t, 4> pickup_poses = {
            se2_create_xya(MIRROR_X(m_color, x_mm - 155), y_mm - 155, MIRROR_A(m_color, -45)),
            se2_create_xya(MIRROR_X(m_color, x_mm + 155), y_mm - 155, MIRROR_A(m_color,  45)),
            se2_create_xya(MIRROR_X(m_color, x_mm + 155), y_mm + 155, MIRROR_A(m_color, 135)),
            se2_create_xya(MIRROR_X(m_color, x_mm - 155), y_mm + 155, MIRROR_A(m_color, 225)),
        };
        strategy_sort_poses_by_distance(
            base_get_robot_pose(&robot.pos), pickup_poses.data(),
            pickup_poses.size(), strategy_distance_to_goal);

        for (size_t i = 0; i < pickup_poses.size(); i++) {
            const int pickup_x_mm = pickup_poses[i].translation.x;
            const int pickup_y_mm = pickup_poses[i].translation.y;
            const int pickup_a_deg = pickup_poses[i].rotation.angle + offset_a_deg;

            NOTICE("Going to %d %d %d", pickup_x_mm, pickup_y_mm, pickup_a_deg);
            if (strategy_goto_avoid(pickup_x_mm, pickup_y_mm, pickup_a_deg, TRAJ_FLAGS_ALL)) {
                break;
            } else if (i == pickup_poses.size() - 1) {
                return false;
            }
        }

        lever_deploy(lever);
        lever_pickup(lever, base_get_robot_pose(&robot.pos), cubes_pose);
        strategy_wait_ms(1300);

        lever_retract(lever);
        if (!strat_lever_is_full(MIRROR_LEVER(m_color, lever_side))) {
            WARNING("No cubes found, waiting for confirmation");
            strategy_wait_ms(200);
            if (!strat_lever_is_full(MIRROR_LEVER(m_color, lever_side))) {
                WARNING("No cubes found confirmed. Abort mission!");
                lever_tidy(lever);
                state.blocks_on_map[blocks_id] = false;
                return false;
            }
        }
        strategy_wait_ms(500);

        if (lever_side == LEVER_SIDE_LEFT) {
            state.lever_full_left = true;
        } else {
            state.lever_full_right = true;
        }
        state.blocks_on_map[blocks_id] = false;
        state.blocks_picked++;

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

    TurnSwitchOn(enum strat_color_t color) : m_color(color) {}

    bool execute(RobotState& state)
    {
        NOTICE("Turning switch on");

        if (!strategy_goto_avoid(MIRROR_X(m_color, 1130), 250, MIRROR_A(m_color, 90),
            TRAJ_END_GOAL_REACHED | TRAJ_END_OPPONENT_NEAR | TRAJ_END_TIMER | TRAJ_END_ALLY_NEAR)) {
            return false;
        }

        state.arms_are_deployed = true;
        strat_push_switch_on(MIRROR_X(m_color, 1130), 50, 120, -120);

        state.switch_on = true;
        return true;
    }
};

struct TurnOpponentSwitchOff : public actions::TurnOpponentSwitchOff {
    enum strat_color_t m_color;

    TurnOpponentSwitchOff(enum strat_color_t color) : m_color(color) {}

    bool execute(RobotState& state)
    {
        NOTICE("Turning opponent switch off O:D");

        if (!strategy_goto_avoid(MIRROR_X(m_color, 1870), 250, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        strat_push_switch_on(MIRROR_X(m_color, 1870), 50, 175, -120);

        state.opponent_panel_on = false;
        state.should_push_opponent_panel = false;
        scara_hold_position(&main_arm, COORDINATE_ARM);


        int res;
        do {
            trajectory_goto_backward_xy_abs(&robot.traj, MIRROR_X(m_color, 1500), 1000);
            res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
        } while (res != TRAJ_END_GOAL_REACHED && res != TRAJ_END_TIMER);

        return true;
    }
};

// struct DeployTheBee : public actions::DeployTheBee {
//     enum strat_color_t m_color;

//     DeployTheBee(enum strat_color_t color) : m_color(color) {}

//     bool execute(RobotState& state)
//     {
//         NOTICE("Gonna deploy the bee!");

//         if (!strategy_goto_avoid(MIRROR_X(m_color, 150), 1850, MIRROR_A(m_color, -90), TRAJ_FLAGS_ALL)) {
//             return false;
//         }

//         state.arms_are_deployed = true;
//         point_t start = {.x = -200.f, .y = MIRROR(m_color, -170.f)};
//         point_t end = {.x = -200.f, .y = MIRROR(m_color, 170.f)};
//         float bee_height = 160.f;
//         strat_push_the_bee(start, end, bee_height);

//         state.bee_deployed = true;
//         return true;
//     }
// };

struct DeployTheBee : public actions::DeployTheBee {
    enum strat_color_t m_color;

    DeployTheBee(enum strat_color_t color) : m_color(color) {}

    bool execute(RobotState& state)
    {
        NOTICE("Gonna deploy the bee!");

        if (!strategy_goto_avoid(MIRROR_X(m_color, 170), 1700, MIRROR_A(m_color, -90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        point_t start = {.x = 120.f, .y = MIRROR(m_color, 60.f)};
        float bee_height = 180.f;
        float forward_motion = -160.f;
        strat_push_the_bee_v2(start, bee_height, forward_motion);

        state.bee_deployed = true;
        return true;
    }
};

struct DepositCubes : actions::DepositCubes {
    enum strat_color_t m_color;

    DepositCubes(enum strat_color_t color, int zone_id)
        : actions::DepositCubes(zone_id), m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = DEPOSIT_ZONE_POSE[construction_zone_id][0];
        const int y_mm = DEPOSIT_ZONE_POSE[construction_zone_id][1];
        int a_deg = DEPOSIT_ZONE_POSE[construction_zone_id][2];
        NOTICE("Depositing cubes at %d %d", x_mm, y_mm);

        enum lever_side_t lever_side = LEVER_SIDE_LEFT;
        lever_t* lever = MIRROR_LEFT_LEVER(m_color);

        if (state.lever_full_left == false) {
            lever_side = LEVER_SIDE_RIGHT;
            lever = MIRROR_RIGHT_LEVER(m_color);
            a_deg += 180;
        }

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, a_deg), TRAJ_FLAGS_ALL)) {
            return false;
        }

        lever_deploy(lever);
        strategy_wait_ms(500);

        se2_t cubes_pose = lever_deposit(lever, base_get_robot_pose(&robot.pos));
        strategy_wait_ms(500);

        lever_push_and_retract(lever);
        strategy_wait_ms(500);

        if (lever_side == LEVER_SIDE_LEFT) {
            state.lever_full_left = false;
        } else {
            state.lever_full_right = false;
        }
        for (int i = 0; i < 5; i++) {
            state.construction_zone[construction_zone_id % 2].cubes_ready[i] = true;
            point_t cube_pos = strategy_cube_pos(cubes_pose, (enum cube_color)i, m_color);
            state.construction_zone[construction_zone_id % 2].cubes_pos[i][0] = cube_pos.x;
            state.construction_zone[construction_zone_id % 2].cubes_pos[i][1] = cube_pos.y;
        }

        return true;
    }
};

void strat_scara_force_shoulder_mode(shoulder_mode_t shoulder_mode)
{
    scara_control_mode_joint(&main_arm);

    main_arm.shoulder_mode = shoulder_mode;
    scara_goto(&main_arm, {170., 0., 295.}, COORDINATE_ARM, {300, 300, 300});
    strategy_wait_ms(500);

    scara_control_mode_cartesian(&main_arm);
}

struct BuildTowerLevel : actions::BuildTowerLevel {
    enum strat_color_t m_color;

    BuildTowerLevel(enum strat_color_t color, int zone_id, int level)
        : actions::BuildTowerLevel(zone_id, level), m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = DEPOSIT_ZONE_POSE[construction_zone_id][0];
        const int y_mm = DEPOSIT_ZONE_POSE[construction_zone_id][1];
        const int a_deg = CONSTRUCTION_HEADING[construction_zone_id];
        NOTICE("Building a tower level %d at %d %d", level, x_mm, y_mm);

        if (level == 0) {
            if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, a_deg), TRAJ_FLAGS_ALL)) {
                return false;
            }

            // Force shoulder position
            const shoulder_mode_t shoulder_mode = CONSTRUCTION_SHOULDER_MODE[construction_zone_id];
            strat_scara_force_shoulder_mode(MIRROR_SHOULDER(m_color, shoulder_mode));
        }

        point_t cube_pos;
        cube_pos.x = state.construction_zone[construction_zone_id % 2].cubes_pos[state.tower_sequence[level]][0];
        cube_pos.y = state.construction_zone[construction_zone_id % 2].cubes_pos[state.tower_sequence[level]][1];

        state.arms_are_deployed = true;
        state.construction_zone[construction_zone_id % 2].cubes_ready[state.tower_sequence[level]] = false;

        if (!strat_pick_cube(cube_pos.x, cube_pos.y)) {
            WARNING("No cube to pick up at %f %f", cube_pos.x, cube_pos.y);
            return false;
        }

        const int tower_x_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][0];
        const int tower_y_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][1];
        if (!strat_deposit_cube(MIRROR_X(m_color, tower_x_mm), tower_y_mm, level)) {
            WARNING("Tower building did not go as expected");
            return false;
        }
        scara_hold_position(&main_arm, COORDINATE_ARM);

        state.construction_zone[construction_zone_id % 2].tower_pos[0] = tower_x_mm;
        state.construction_zone[construction_zone_id % 2].tower_pos[1] = tower_y_mm;
        state.construction_zone[construction_zone_id % 2].tower_level += 1;
        return true;
    }
};

void strat_collect_wastewater(enum strat_color_t color, float heading)
{
    const int SHAKE_AMPLITUDE_DEG = 7;
    ballgun_deploy_charge(&main_ballgun);
    strategy_wait_ms(500);

    ballgun_charge(&main_ballgun);
    strategy_wait_ms(500);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading));
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE /*TRAJ_FLAGS_ROTATION*/);

    // open dispenser
    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading + SHAKE_AMPLITUDE_DEG+3));
    strategy_wait_ms(500);

    // move closer
    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading));
    strategy_wait_ms(500);
    trajectory_d_rel(&robot.traj, 10);
    strategy_wait_ms(500);

    for (auto i = 0; i < 3; i++) {
        trajectory_a_abs(&robot.traj, MIRROR_A(color, heading - SHAKE_AMPLITUDE_DEG));
        strategy_wait_ms(500);
        trajectory_a_abs(&robot.traj, MIRROR_A(color, heading + SHAKE_AMPLITUDE_DEG));
        strategy_wait_ms(500);
    }

    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading));
    strategy_wait_ms(500);

    trajectory_d_rel(&robot.traj, -100);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    ballgun_tidy(&main_ballgun);
}

struct EmptyMonocolorWasteWaterCollector : actions::EmptyMonocolorWasteWaterCollector {
    enum strat_color_t m_color;

    EmptyMonocolorWasteWaterCollector(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 0 + 240;
        const int y_mm = 840;
        NOTICE("Emptying monocolor waste water collector at %d %d", x_mm, y_mm);

        se2_t pos;
        lever_deposit(&left_lever, pos);
        lever_deposit(&right_lever, pos);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm + 100), y_mm, MIRROR_A(m_color, -180), TRAJ_FLAGS_ALL)) {
            return false;
        }

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, -150), TRAJ_FLAGS_ALL)) {
            return false;
        }

        strat_collect_wastewater(m_color, -180);

        state.ballgun_state = BallgunState::CHARGED_MONOCOLOR;
        state.wastewater_monocolor_full = false;

        return true;
    }
};

struct EmptyMulticolorWasteWaterCollector : actions::EmptyMulticolorWasteWaterCollector {
    enum strat_color_t m_color;

    EmptyMulticolorWasteWaterCollector(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 2390;
        const int y_mm = 2000 - 240;
        NOTICE("Emptying multicolor waste water collector at %d %d", x_mm, y_mm);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm - 100, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, 120), TRAJ_FLAGS_ALL)) {
            return false;
        }

        strat_collect_wastewater(m_color, 90);

        state.ballgun_state = BallgunState::CHARGED_MULTICOLOR;
        state.wastewater_multicolor_full = false;

        return true;
    }
};

bool strat_fill_watertower(void)
{
    ballgun_deploy(&main_ballgun);
    strategy_wait_ms(500);

    ballgun_spin(&main_ballgun);
    strategy_wait_ms(500);

    ballgun_fire(&main_ballgun);

    const auto remaining = (GAME_DURATION * 1000) - trajectory_get_time_ms();
    const auto delay = std::max(std::min(3000, remaining), 10);
    strategy_wait_ms(delay);

    ballgun_tidy(&main_ballgun);

    if (delay < 500) {
        return false; // not enough time
    }

    return true;
}

struct FireBallGunIntoWaterTower : actions::FireBallGunIntoWaterTower {
    enum strat_color_t m_color;

    FireBallGunIntoWaterTower(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 430; // tuned by experience
        const int y_mm = 410;
        const int heading_deg = 247;
        NOTICE("Filling water tower from %d %d", x_mm, y_mm);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, heading_deg), TRAJ_FLAGS_ALL)) {
            return false;
        }

        ball_sense_reset_count();

        auto res = strat_fill_watertower();
        if (res) {
            state.ballgun_state = BallgunState::IS_EMPTY;
        }

        state.balls_in_watertower += ball_sense_count();

        return res;
    }
};

void strat_fill_wastewater_treatment_plant(void)
{
    ballgun_tidy(&main_ballgun);
    ballgun_deploy_fully(&main_ballgun);
    strategy_wait_ms(1000);

    // pre-spin "ball-accelerator" motor with ~5V to ensure that it's not blocking
    ball_accelerator_voltage(5.0f);
    strategy_wait_ms(500);
    ballgun_slowfire(&main_ballgun);
    strategy_wait_ms(2000);

    ballgun_tidy(&main_ballgun);
}

struct FireBallGunIntoWasteWaterTreatmentPlant : actions::FireBallGunIntoWasteWaterTreatmentPlant {
    enum strat_color_t m_color;

    FireBallGunIntoWasteWaterTreatmentPlant(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 2360;
        const int y_mm = 1758;
        const int heading_deg = 170;
        NOTICE("Filling waste water treatment plant from %d %d", x_mm, y_mm);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, heading_deg), TRAJ_FLAGS_ALL)) {
            return false;
        }

        ball_sense_reset_count();
        strat_fill_wastewater_treatment_plant();

        state.ballgun_state = BallgunState::IS_EMPTY;
        state.balls_in_wastewater_treatment_plant += ball_sense_count() / 2;

        NOTICE("Getting out of drop zone");
        int res;
        do {
            const int retreat_x_mm = 2390;
            const int retreat_y_mm = 2000 - 240 - 200;
            trajectory_goto_backward_xy_abs(&robot. traj, MIRROR_X(m_color, retreat_x_mm), retreat_y_mm);
            res = trajectory_wait_for_end(TRAJ_FLAGS_ALL);
        } while (res != TRAJ_END_GOAL_REACHED && res != TRAJ_END_TIMER);

        return true;
    }
};

void strategy_read_color_sequence(RobotState& state)
{
    int tower_sequence_len = sizeof(state.tower_sequence) / sizeof(enum cube_color);
    messagebus_topic_t* colors_topic = messagebus_find_topic_blocking(&bus, "/colors");

    messagebus_topic_read(colors_topic, &state.tower_sequence[0], tower_sequence_len);

    if (state.tower_sequence[0] != CUBE_UNKNOWN &&
        state.tower_sequence[1] != CUBE_UNKNOWN &&
        state.tower_sequence[2] != CUBE_UNKNOWN) {
        NOTICE("Received a valid color sequence");
        state.tower_sequence_known = true;
    }

    cube_color_fill_unknown(&state.tower_sequence[0], tower_sequence_len);

    NOTICE("Tower sequence is: %s %s %s %s %s",
           cube_color_name(state.tower_sequence[0]),
           cube_color_name(state.tower_sequence[1]),
           cube_color_name(state.tower_sequence[2]),
           cube_color_name(state.tower_sequence[3]),
           cube_color_name(state.tower_sequence[4]));
}

void strategy_order_play_game(enum strat_color_t color, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;

    BeeGoal bee_goal;
    PickupCubesGoal pickup_cubes_goal[2] = {PickupCubesGoal(1), PickupCubesGoal(2)};
    WaterTowerGoal watertower_goal;
    BuildTowerGoal build_tower_goal[2] = {BuildTowerGoal(0), BuildTowerGoal(1)};
    goap::Goal<RobotState>* goals[] = {
        &pickup_cubes_goal[0],
        &pickup_cubes_goal[1],
        &bee_goal,
        &build_tower_goal[0],
        &build_tower_goal[1],
        &watertower_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    PickupCubes pickup_cubes[2] = {
        PickupCubes(color, 1), PickupCubes(color, 2),
    };
    DeployTheBee deploy_the_bee(color);
    DepositCubes deposit_cubes[2] = {
        DepositCubes(color, 0), DepositCubes(color, 1),
    };
    BuildTowerLevel build_tower_lvl[2][4] = {
        {
            BuildTowerLevel(color, 0, 0), BuildTowerLevel(color, 0, 1),
            BuildTowerLevel(color, 0, 2), BuildTowerLevel(color, 0, 3),
        },
        {
            BuildTowerLevel(color, 1, 0), BuildTowerLevel(color, 1, 1),
            BuildTowerLevel(color, 1, 2), BuildTowerLevel(color, 1, 3),
        },
    };
    EmptyMonocolorWasteWaterCollector empty_wastewater(color);
    FireBallGunIntoWaterTower fill_watertower(color);

    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};

    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &pickup_cubes[0],
        &pickup_cubes[1],
        &deploy_the_bee,
        &deposit_cubes[0],
        &deposit_cubes[1],
        &build_tower_lvl[0][0],
        &build_tower_lvl[0][1],
        &build_tower_lvl[0][2],
        &build_tower_lvl[0][3],
        &build_tower_lvl[1][0],
        &build_tower_lvl[1][1],
        &build_tower_lvl[1][2],
        &build_tower_lvl[1][3],
        &empty_wastewater,
        &fill_watertower,
    };

    static goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    lever_retract(&right_lever);
    lever_retract(&left_lever);
    ballgun_tidy(&main_ballgun);
    wrist_set_horizontal(&main_wrist);

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        messagebus_topic_publish(state_topic, &state, sizeof(state));
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(color, 200), 520, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();
    trajectory_game_timer_reset();
    strategy_read_color_sequence(state);

    NOTICE("Starting game...");
    while (!trajectory_game_has_ended()) {
        for (auto goal : goals) {
            int len = planner.plan(state, *goal, path, max_path_len);
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

    // Avoid burning the ball gun by deploying it
    ballgun_deploy(&main_ballgun);

    NOTICE("Game ended!");
    while (true) {
        strategy_wait_ms(1000);
    }
}

void strategy_chaos_play_game(enum strat_color_t color, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;

    SwitchGoal switch_goal;
    PickupCubesGoal pickup_cubes_goal[2] = {PickupCubesGoal(0), PickupCubesGoal(3)};
    WasteWaterGoal wastewater_plant_goal;
    BuildTowerGoal build_tower_goal[2] = {BuildTowerGoal(2), BuildTowerGoal(3)};
    OpponentPanelGoal opponent_panel_goal;
    goap::Goal<RobotState>* goals[] = {
        &switch_goal,
        &pickup_cubes_goal[0],
        &pickup_cubes_goal[1],
        &wastewater_plant_goal,
        &build_tower_goal[0],
        &build_tower_goal[1],
        &opponent_panel_goal, // muahaha
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    PickupCubes pickup_cubes[2] = {
        PickupCubes(color, 0), PickupCubes(color, 3),
    };
    TurnSwitchOn turn_switch_on(color);
    EmptyMulticolorWasteWaterCollector empty_wastewater_multicolor(color);
    FireBallGunIntoWasteWaterTreatmentPlant fill_wasterwater_plant(color);
    DepositCubes deposit_cubes[2] = {
        DepositCubes(color, 2), DepositCubes(color, 3),
    };
    BuildTowerLevel build_tower_lvl[2][4] = {
        {
            BuildTowerLevel(color, 2, 0), BuildTowerLevel(color, 2, 1),
            BuildTowerLevel(color, 2, 2), BuildTowerLevel(color, 2, 3),
        },
        {
            BuildTowerLevel(color, 3, 0), BuildTowerLevel(color, 3, 1),
            BuildTowerLevel(color, 3, 2), BuildTowerLevel(color, 3, 3),
        },
    };
    TurnOpponentSwitchOff turn_opponent_switch_off(color);

    const int max_path_len = 10;
    goap::Action<RobotState> *path[max_path_len] = {nullptr};

    goap::Action<RobotState> *actions[] = {
        &index_arms,
        &retract_arms,
        &pickup_cubes[0],
        &pickup_cubes[1],
        &turn_switch_on,
        &empty_wastewater_multicolor,
        &fill_wasterwater_plant,
        &deposit_cubes[0],
        &deposit_cubes[1],
        &build_tower_lvl[0][0],
        &build_tower_lvl[0][1],
        &build_tower_lvl[0][2],
        &build_tower_lvl[0][3],
        &build_tower_lvl[1][0],
        &build_tower_lvl[1][1],
        &build_tower_lvl[1][2],
        &build_tower_lvl[1][3],
        &turn_opponent_switch_off,
    };

    static goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    lever_retract(&right_lever);
    lever_retract(&left_lever);
    ballgun_tidy(&main_ballgun);
    wrist_set_horizontal(&main_wrist);

    NOTICE("Getting arms ready...");
    int len = planner.plan(state, init_goal, path, max_path_len);
    for (int i = 0; i < len; i++) {
        path[i]->execute(state);
        messagebus_topic_publish(state_topic, &state, sizeof(state));
    }

    /* Autoposition robot */
    wait_for_autoposition_signal();
    NOTICE("Positioning robot");

    robot.base_speed = BASE_SPEED_INIT;
    strategy_auto_position(MIRROR_X(color, 200), 180, MIRROR_A(color, -90), color);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
    trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

    robot.base_speed = BASE_SPEED_FAST;

    NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
           position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));

    /* Wait for starter to begin */
    wait_for_starter();

    trajectory_game_timer_reset();
    strategy_read_color_sequence(state);

    NOTICE("Moving out of Order's way. Time for Chaos!");
    trajectory_d_rel(&robot.traj, -400);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    NOTICE("Starting game...");

    state.should_push_opponent_panel = true;

    while (!trajectory_game_has_ended()) {
        for (auto goal : goals) {
            int len = planner.plan(state, *goal, path, max_path_len);
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

        if (trajectory_get_time() > 70) {
            NOTICE("Asking GOAP to shut down this panel.");
            state.opponent_panel_on = true;
        }

        update_panel_state_from_uwb(state);
        strategy_wait_ms(10);
    }

    // Avoid burning the ball gun by deploying it
    ballgun_deploy(&main_ballgun);

    // Check that opponent didn't switch off our panel
    strategy_wait_ms(3000);
    update_panel_state_from_uwb(state);

    NOTICE("Game ended!");
    while (true) {
        strategy_wait_ms(1000);
    }
}

void strategy_play_game(void *p)
{
    (void) p;
    chRegSetThreadName("strategy");

    NOTICE("Strategy starting...");

    /* Prepare state publisher */
    RobotState state;

    static messagebus_topic_t state_topic;
    static MUTEX_DECL(state_lock);
    static CONDVAR_DECL(state_condvar);
    static RobotState state_topic_content;
    messagebus_topic_init(&state_topic, &state_lock, &state_condvar, &state_topic_content, sizeof(state));
    messagebus_advertise_topic(&bus, &state_topic, "/state");

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
