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
#include "config.h"
#include "control_panel.h"
#include "main.h"

#include "strategy.h"
#include "strategy/actions.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy/score_counter.h"
#include "strategy/color_sequence_server.h"


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

        scara_goto(&main_arm, {-100., MIRROR(m_color, 120.), 295.}, COORDINATE_ROBOT, {300, 300, 300});
        strategy_wait_ms(500);
        scara_goto(&main_arm, {20., MIRROR(m_color, 90.), 295.}, COORDINATE_ROBOT, {300, 300, 300});
        strategy_wait_ms(500);

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

bool strat_check_distance_to_hand_lower_than(float expected_value)
{
    bool success = true;

    float distance_to_tower;
    messagebus_topic_t* topic = messagebus_find_topic_blocking(&bus, "/hand_distance");

    if (messagebus_topic_read(topic, &distance_to_tower, sizeof(distance_to_tower))) {
        WARNING("Hand distance sensor is not publishing");
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

    scara_goto(&main_arm, {x, y, 80}, COORDINATE_TABLE, {300, 300, 300});
    arm_traj_wait_for_end();

    scara_goto(&main_arm, {x, y, 60}, COORDINATE_TABLE, {200, 200, 200});
    arm_traj_wait_for_event(ARM_READY | ARM_BLOCKED_Z);

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
        // strategy_sort_poses_by_distance(
        //     base_get_robot_pose(&robot.pos), pickup_poses.data(),
        //     pickup_poses.size(), strategy_distance_to_goal);

        // Ugly hack. TODO: debug later the offset. Don't judge me.
        int i = std::min(blocks_id, 3); // don't segfault please
        // for (size_t i = 0; i < pickup_poses.size(); i++) {
            const int pickup_x_mm = pickup_poses[i].translation.x;
            const int pickup_y_mm = pickup_poses[i].translation.y;
            const int pickup_a_deg = pickup_poses[i].rotation.angle + offset_a_deg;

            NOTICE("Going to %d %d %d", pickup_x_mm, pickup_y_mm, pickup_a_deg);
            if (strategy_goto_avoid(pickup_x_mm, pickup_y_mm, pickup_a_deg, TRAJ_FLAGS_ALL)) {
                // go to same position, this time should be slightly more precise
                strategy_goto_avoid(pickup_x_mm, pickup_y_mm, pickup_a_deg, TRAJ_FLAGS_ALL);
                // break;
            // } else if (i == pickup_poses.size() - 1) {
            } else {
                return false;
            }
        // }

        lever_deploy(lever);
        lever_pickup(lever, base_get_robot_pose(&robot.pos), cubes_pose);
        strategy_wait_ms(1000);

        lever_retract(lever);
        strategy_wait_ms(500);

        if (lever_side == LEVER_SIDE_LEFT) {
            state.lever_full_left = true;
        } else {
            state.lever_full_right = true;
        }
        state.blocks_on_map[blocks_id] = false;

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

        if (!strategy_goto_avoid(MIRROR_X(m_color, 1130), 250, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        state.arms_are_deployed = true;
        strat_push_switch_on(MIRROR_X(m_color, 1130), 50, 120, -120);

        state.switch_on = true;
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
        float forward_motion = -170.f;
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
        const int x_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][0];
        const int y_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][1];
        NOTICE("Depositing cubes at %d %d", x_mm, y_mm);

        enum lever_side_t lever_side = LEVER_SIDE_LEFT;
        lever_t* lever = MIRROR_LEFT_LEVER(m_color);
        int a_deg = -135;

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
            state.construction_zone[construction_zone_id].cubes_ready[i] = true;
            point_t cube_pos = strategy_cube_pos(cubes_pose, (enum cube_color)i, m_color);
            state.construction_zone[construction_zone_id].cubes_pos[i][0] = cube_pos.x;
            state.construction_zone[construction_zone_id].cubes_pos[i][1] = cube_pos.y;
        }

        return true;
    }
};

struct BuildTowerLevel : actions::BuildTowerLevel {
    enum strat_color_t m_color;

    BuildTowerLevel(enum strat_color_t color, int zone_id, int level)
        : actions::BuildTowerLevel(zone_id, level), m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][0];
        const int y_mm = CONSTRUCTION_ZONE_POS[construction_zone_id][1];
        NOTICE("Building a tower level %d at %d %d", level, x_mm, y_mm);

        if (level == 0) {
            if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, -215), TRAJ_FLAGS_ALL)) {
                return false;
            }
        }

        point_t cube_pos;
        cube_pos.x = state.construction_zone[construction_zone_id].cubes_pos[state.tower_sequence[level]][0];
        cube_pos.y = state.construction_zone[construction_zone_id].cubes_pos[state.tower_sequence[level]][1];

        state.arms_are_deployed = true;
        state.construction_zone[construction_zone_id].cubes_ready[state.tower_sequence[level]] = false;

        if (!strat_pick_cube(cube_pos.x, cube_pos.y)) {
            WARNING("No cube to pick up at %d %d", cube_pos.x, cube_pos.y);
            return false;
        }

        const int tower_x_mm = x_mm + 0;
        const int tower_y_mm = y_mm - 220;
        if (!strat_deposit_cube(MIRROR_X(m_color, tower_x_mm), tower_y_mm, level)) {
            WARNING("Tower building did not go as expected");
            return false;
        }
        scara_hold_position(&main_arm, COORDINATE_ARM);

        state.construction_zone[construction_zone_id].tower_pos[0] = tower_x_mm;
        state.construction_zone[construction_zone_id].tower_pos[1] = tower_y_mm;
        state.construction_zone[construction_zone_id].tower_level += 1;
        return true;
    }
};

void strat_collect_wastewater(enum strat_color_t color, float heading)
{
    const int shake_amplitude = 7;
    ballgun_tidy(&main_ballgun);
    ballgun_deploy(&main_ballgun);
    strategy_wait_ms(1000);

    ballgun_charge(&main_ballgun);

    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading - shake_amplitude));
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE /*TRAJ_FLAGS_ROTATION*/);

    for (auto i = 0; i < 6; i++) {
        trajectory_a_abs(&robot.traj, MIRROR_A(color, heading + shake_amplitude));
        strategy_wait_ms(500);
        trajectory_a_abs(&robot.traj, MIRROR_A(color, heading - shake_amplitude));
        strategy_wait_ms(500);
    }

    trajectory_a_abs(&robot.traj, MIRROR_A(color, heading));

    strategy_wait_ms(2000);

    trajectory_d_rel(&robot.traj, -50);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    ballgun_tidy(&main_ballgun);
}

struct EmptyMonocolorWasteWaterCollector : actions::EmptyMonocolorWasteWaterCollector {
    enum strat_color_t m_color;

    EmptyMonocolorWasteWaterCollector(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 0 + 265;
        const int y_mm = 840;
        NOTICE("Emptying monocolor waste water collector at %d %d", x_mm, y_mm);

        se2_t pos;
        lever_deposit(&left_lever, pos);
        lever_deposit(&right_lever, pos);

        state.lever_full_left = false;
        state.lever_full_right = false;

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm + 100), y_mm, MIRROR_A(m_color, -180), TRAJ_FLAGS_ALL)) {
            return false;
        }

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, -90), TRAJ_FLAGS_ALL)) {
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
        const int y_mm = 2000 - 265;
        NOTICE("Emptying multicolor waste water collector at %d %d", x_mm, y_mm);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm - 100, MIRROR_A(m_color, 90), TRAJ_FLAGS_ALL)) {
            return false;
        }

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, -180), TRAJ_FLAGS_ALL)) {
            return false;
        }

        strat_collect_wastewater(m_color, 90);

        state.ballgun_state = BallgunState::CHARGED_MULTICOLOR;
        state.wastewater_multicolor_full = false;

        return true;
    }
};

void strat_fill_watertower(void)
{
    ballgun_tidy(&main_ballgun);
    ballgun_deploy(&main_ballgun);
    strategy_wait_ms(500);

    ballgun_fire(&main_ballgun);

    const auto remaining = (GAME_DURATION * 1000) - trajectory_get_time_ms();
    const auto delay = std::max(std::min(2000, remaining), 10);
    strategy_wait_ms(delay);

    // ballgun_tidy(&main_ballgun);
    // keep ballgun out to avoid heating servo
    ballgun_arm(&main_ballgun);
}

struct FireBallGunIntoWaterTower : actions::FireBallGunIntoWaterTower {
    enum strat_color_t m_color;

    FireBallGunIntoWaterTower(enum strat_color_t color)
        : m_color(color) {}

    bool execute(RobotState &state) {
        const int x_mm = 400;
        const int y_mm = 344;
        const int heading_deg = 247;
        NOTICE("Filling water tower from %d %d", x_mm, y_mm);

        if (!strategy_goto_avoid(MIRROR_X(m_color, x_mm), y_mm, MIRROR_A(m_color, heading_deg), TRAJ_FLAGS_ALL)) {
            return false;
        }

        strat_fill_watertower();

        state.ballgun_state = BallgunState::IS_EMPTY;
        state.balls_in_watertower += 5;

        return true;
    }
};

void strat_fill_wastewater_treatment_plant(void)
{
    ballgun_tidy(&main_ballgun);
    ballgun_deploy_fully(&main_ballgun);
    strategy_wait_ms(1000);

    // TODO: pre-spin "ball-accelerator" motor with ~5V to ensure that it's not blocking
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

        strat_fill_wastewater_treatment_plant();

        state.ballgun_state = BallgunState::IS_EMPTY;
        state.balls_in_wastewater_treatment_plant += 4;

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
    PickupCubesGoal pickup_cubes_goal;
    WaterTowerGoal watertower_goal;
    BuildTowerGoal build_tower_goal[2] = {BuildTowerGoal(0), BuildTowerGoal(1)};
    goap::Goal<RobotState>* goals[] = {
        &bee_goal,
        &pickup_cubes_goal,
        &build_tower_goal[0],
        &build_tower_goal[1],
        &watertower_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    PickupCubes pickup_cubes[2] = {
        PickupCubes(color, 0), PickupCubes(color, 1),
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

    trajectory_a_rel(&robot.traj, MIRROR(color, 45));
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    trajectory_d_rel(&robot.traj, -150);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    NOTICE("Starting game...");
    auto goals_are_reached = [&goals](const RobotState& state) {
        for (auto goal : goals) {
            if (!goal->is_reached(state)) {
                return false;
            }
        }
        return true;
    };
    while (!goals_are_reached(state)) {
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
        if (trajectory_game_has_ended()) {
            break;
        }
    }

    // Avoid burning the ball gun by deploying it
    ballgun_deploy(&main_ballgun);

    while (true) {
        NOTICE("Game ended!");
        strategy_wait_ms(1000);
    }
}

void strategy_chaos_play_game(enum strat_color_t color, RobotState& state)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    InitGoal init_goal;

    SwitchGoal switch_goal;
    PickupCubesGoal pickup_cubes_goal;
    WasteWaterGoal wastewater_plant_goal;
    goap::Goal<RobotState>* goals[] = {
        &switch_goal,
        &wastewater_plant_goal,
        // &pickup_cubes_goal,
    };

    IndexArms index_arms;
    RetractArms retract_arms(color);
    PickupCubes pickup_cubes[2] = {
        PickupCubes(color, 2), PickupCubes(color, 3),
    };
    TurnSwitchOn turn_switch_on(color);
    EmptyMulticolorWasteWaterCollector empty_wastewater_multicolor(color);
    FireBallGunIntoWasteWaterTreatmentPlant fill_wasterwater_plant(color);

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
    };

    static goap::Planner<RobotState> planner(actions, sizeof(actions) / sizeof(actions[0]));

    lever_retract(&right_lever);
    lever_retract(&left_lever);
    ballgun_tidy(&main_ballgun);

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

    strategy_wait_ms(2000); /* Wait for Order to get out */
    trajectory_d_rel(&robot.traj, -400);
    trajectory_wait_for_end(TRAJ_FLAGS_SHORT_DISTANCE);

    NOTICE("Starting game...");
    auto goals_are_reached = [&goals](const RobotState& state) {
        for (auto goal : goals) {
            if (!goal->is_reached(state)) {
                return false;
            }
        }
        return true;
    };
    while (!goals_are_reached(state)) {
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
        if (trajectory_game_has_ended()) {
            break;
        }
    }

    // Avoid burning the ball gun by deploying it
    ballgun_deploy(&main_ballgun);

    while (true) {
        NOTICE("Game ended!");
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
    messagebus_topic_init(&state_topic, &state_lock, &state_condvar, &state, sizeof(state));
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
