// TODO: Enable this once map is ported
#define USE_MAP 0

#include <array>
#include <thread>

#include <absl/flags/flag.h>

#include <aversive/blocking_detection_manager/blocking_detection_manager.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <error/error.h>
#include <goap/goap.hpp>

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/autoposition.h"
#include "robot_helpers/motor_helpers.h"
#include "base/base_controller.h"

#if USE_MAP
#include "base/map.h"
#include "base/map_server.h"
#endif

#include "config.h"
#include "control_panel.h"
#include "main.h"

#include "strategy.h"
#include "strategy/color.h"
#include "strategy/actions.h"
#include "strategy/goals.h"
#include "strategy/state.h"

ABSL_FLAG(bool, strat_autoposition, false, "Automatically calibrate the robot's position on the table. Not intended for simulation.");
ABSL_FLAG(bool, strat_wait_for_starter, false, "Wait for starter input before going off.");

using namespace std::chrono_literals;

// TODO(antoinealb): Move GOAP defines to something shared with unit tests
const int MAX_GOAP_PATH_LEN = 10;

static goap::Planner<StrategyState, GOAP_SPACE_SIZE> planner;

static enum strat_color_t wait_for_color_selection();
static void wait_for_autoposition_signal();
static void wait_for_starter();
void strategy_play_game();

static enum strat_color_t wait_for_color_selection()
{
    strat_color_t color = YELLOW;

    while (!control_panel_button_is_pressed(BUTTON_YELLOW) && !control_panel_button_is_pressed(BUTTON_GREEN)) {
        control_panel_set(LED_YELLOW);
        control_panel_set(LED_GREEN);
        std::this_thread::sleep_for(100ms);
        control_panel_clear(LED_YELLOW);
        control_panel_clear(LED_GREEN);
        std::this_thread::sleep_for(100ms);
    }

    if (control_panel_button_is_pressed(BUTTON_GREEN)) {
        control_panel_clear(LED_YELLOW);
        control_panel_set(LED_GREEN);
        color = YELLOW;
        NOTICE("Color set to yellow");
    } else {
        control_panel_set(LED_YELLOW);
        control_panel_clear(LED_GREEN);
        color = BLUE;
        NOTICE("Color set to blue");
    }

    return color;
}

static void wait_for_starter()
{
    const long STARTER_MIN_ARMED_TIME_MS = 1000;
    unsigned long starter_armed_time_ms;

    control_panel_clear(LED_READY);

    while (true) {
        starter_armed_time_ms = 0;

        /* Wait for a rising edge */
        while (control_panel_read(STARTER)) {
            std::this_thread::sleep_for(10ms);
        }
        while (!control_panel_read(STARTER)) {
            std::this_thread::sleep_for(10ms);
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

static void wait_for_autoposition_signal()
{
    wait_for_color_selection();
}

goals::LighthouseEnabled lighthouse_enabled;
actions::EnableLighthouse enable_lighthouse;
goals::WindsocksUp windsocks_raised;
actions::BackwardReefPickup backward_reef_pickup;

actions::RaiseWindsock windsocks[2] = {{0}, {1}};

std::vector<actions::NamedAction<StrategyState>*> strategy_get_actions()
{
    // Put all the actions the robot should consider in here.
    return {{
        &enable_lighthouse,
        &windsocks[0],
        &windsocks[1],
        &backward_reef_pickup,
    }};
}

void strategy_order_play_game(StrategyState& state, enum strat_color_t color)
{
    messagebus_topic_t* state_topic = messagebus_find_topic_blocking(&bus, "/state");

    goap::Action<StrategyState>* path[MAX_GOAP_PATH_LEN] = {nullptr};

    std::array<goap::Goal<StrategyState>*, 2> goals = {&lighthouse_enabled, &windsocks_raised};

    /* Autoposition robot */
    if (absl::GetFlag(FLAGS_strat_autoposition)) {
        wait_for_autoposition_signal();
        NOTICE("Positioning robot");

        robot.base_speed = BASE_SPEED_INIT;
        strategy_auto_position(MIRROR_X(color, 250), 450, MIRROR_A(color, -90), color);

        trajectory_a_abs(&robot.traj, MIRROR_A(color, 180));
        trajectory_wait_for_end(TRAJ_END_GOAL_REACHED);

        robot.base_speed = BASE_SPEED_FAST;
        NOTICE("Robot positioned at x: %d[mm], y: %d[mm], a: %d[deg]",
               position_get_x_s16(&robot.pos), position_get_y_s16(&robot.pos), position_get_a_deg_s16(&robot.pos));
    }

    /* Wait for starter to begin */
    if (absl::GetFlag(FLAGS_strat_wait_for_starter)) {
        wait_for_starter();
    } else {
        std::this_thread::sleep_for(2s);
    }

    trajectory_game_timer_reset();

    NOTICE("Starting game...");

    auto actions = strategy_get_actions();
    std::vector<goap::Action<StrategyState>*> action_ptrs;
    std::transform(actions.begin(), actions.end(), std::back_inserter(action_ptrs),
                   [](actions::NamedAction<StrategyState>* p) { return p; });

    // Always find a non complete goal, find actions to fulfill it, then apply
    // those actions.
    while (!trajectory_game_has_ended()) {
        for (auto* goal : goals) {
            int len = planner.plan(state, *goal,
                                   action_ptrs.data(),
                                   action_ptrs.size(), path, MAX_GOAP_PATH_LEN);
            for (int i = 0; i < len; i++) {
                bool success = path[i]->execute(state);
                messagebus_topic_publish(state_topic, &state, sizeof(state));
                if (!success) {
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
    }

    NOTICE("Deploying flags");
    WARNING("Unimplemented");

    state.robot.flags_deployed = true;
    messagebus_topic_publish(state_topic, &state, sizeof(state));

    NOTICE("Game ended!");
}

void strategy_play_game()
{
    NOTICE("Strategy starting...");

    /* Prepare state publisher */
    StrategyState state = initial_state();

    static TOPIC_DECL(state_topic, StrategyState);
    messagebus_advertise_topic(&bus, &state_topic.topic, "/state");

    NOTICE("Waiting for color selection...");
    //auto color = wait_for_color_selection();
#if USE_MAP
    map_server_start(color);
#endif

    strategy_order_play_game(state, YELLOW);
}
