#include <string.h>
#include <iostream>

#include "msgbus_port.h"
#include "simulation.h"

#include "strategy.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy/score.h"
#include "strategy_impl/game.h"

namespace sim {
RobotState state;
enum strat_color_t color = YELLOW;
strategy_context_t* ctx = strategy_simulated_impl(color);

GAME_GOALS_CHAOS(goals, goal_names, goal_count);
GAME_ACTIONS_CHAOS(actions, action_count, ctx);

static TOPIC_DECL(state_topic, RobotState);

void reset(void)
{
    sim::state = initial_state();
    sim::state.arms_are_indexed = true;
    position_set(&sim::ctx->robot->pos, MIRROR_X(sim::color, 250), 450, -90);

    std::cout << "Reset to factory settings: done" << std::endl;
    std::cout << "State pucks ";
    size_t num_pucks_scale = sizeof(sim::state.puck_in_scale) / sizeof(PuckColor);
    for (int i = 0; i < num_pucks_scale; i++) {
        std::cout << sim::state.puck_in_scale[i] << " ";
    }
    std::cout << std::endl;
}

int count_score(const RobotState& state)
{
    int score = 0;
    score += score_count_classified_atoms(state);
    score += score_count_accelerator(state);
    score += score_count_goldenium(state);
    score += score_count_experiment(state);
    score += score_count_electron(state);
    score += score_count_scale(state);
    return score;
}

void run_goal(goap::Goal<RobotState>* goal, std::string name)
{
    goap::Action<RobotState>* path[MAX_GOAP_PATH_LEN] = {nullptr};
    static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;
    int len = planner.plan(sim::state, *goal, sim::actions, sim::action_count, path, MAX_GOAP_PATH_LEN);
    std::cout << "Found a path of length " << std::to_string(len) << " to achieve the " << name << " goal" << std::endl;
    messagebus_topic_publish(&sim::state_topic.topic, &sim::state, sizeof(sim::state));
    for (int i = 0; i < len; i++) {
        std::cout << std::endl
                  << "#" << std::to_string(i);
        bool success = path[i]->execute(sim::state);
        messagebus_topic_publish(&sim::state_topic.topic, &sim::state, sizeof(sim::state));
        if (success == false) {
            std::cout << "Failed to execute action #" << std::to_string(i) << std::endl;
            break; // Break on failure
        }
    }
    std::cout << "Score estimate: " << std::to_string(count_score(state)) << std::endl;
}
} // namespace sim

int main(int argc, char* argv[])
{
    /* Create the message bus. */
    condvar_wrapper_t bus_sync = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};
    messagebus_init(&bus, &bus_sync, &bus_sync);

    simulation_init();

    messagebus_advertise_topic(&bus, &sim::state_topic.topic, "/state");

    if (argc != 2) {
        std::cout << "Usage: simulator y|v" << std::endl;
        return 0;
    }

    if (!strcmp(argv[1], "v")) {
        std::cout << "Playing in violet" << std::endl;
        sim::color = VIOLET;
        sim::ctx->color = sim::color;
    } else {
        std::cout << "Playing in yellow" << std::endl;
    }

    sim::reset();
    while (true) {
        std::cout << "----------------------------------" << std::endl;
        std::cout << "> ";

        std::string line;
        std::getline(std::cin, line);

        // CTRL-D was pressed -> exit
        if (line[0] == 'q') {
            std::cout << "Exiting..." << std::endl;
            return 0;
        }

        if (!strcmp(line.c_str(), "help")) {
            std::cout << "Welcome to the help menu, here are the commands available:" << std::endl;
            std::cout << "- reset [Reset game state]" << std::endl;
            std::cout << "- pos [Show current bot position]" << std::endl;
            std::cout << "- all [Run all goals]" << std::endl;
            for (size_t i = 0; i < sim::goal_count; i++) {
                std::cout << "- " << sim::goal_names[i] << std::endl;
            }
            continue;
        }

        if (!strcmp(line.c_str(), "reset")) {
            sim::reset();
            continue;
        }

        if (!strcmp(line.c_str(), "pos")) {
            publish_pos(sim::ctx);
            continue;
        }

        if (!strcmp(line.c_str(), "all")) {
            for (size_t i = 0; i < sim::goal_count; i++) {
                sim::run_goal(sim::goals[i], sim::goal_names[i]);
                std::cout << "----------------------------------" << std::endl;
            }
            std::cout << "Ran through all goals" << std::endl;
            continue;
        }

        goap::Goal<RobotState>* goal = nullptr;
        for (size_t i = 0; i < sim::goal_count; i++) {
            if (!strcmp(line.c_str(), sim::goal_names[i])) {
                goal = sim::goals[i];
            }
        }
        if (goal == nullptr) {
            std::cout << "Unknown goal " << line << std::endl;
            continue;
        }
        sim::run_goal(goal, line);
    }
    return 0;
}
