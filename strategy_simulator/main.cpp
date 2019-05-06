#include <string.h>
#include <iostream>

#include "msgbus_port.h"
#include "simulation.h"

#include "strategy.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy_impl/game.h"

namespace sim {
RobotState state;
enum strat_color_t color = YELLOW;
strategy_context_t* ctx = strategy_simulated_impl(color);

GAME_GOALS_CHAOS(goals, goal_names, goal_count);
GAME_ACTIONS_CHAOS(actions, action_count, ctx);

const int max_path_len = 10;

void reset(void)
{
    sim::state = initial_state();
    sim::state.arms_are_indexed = true;
    position_set(&sim::ctx->robot->pos, MIRROR_X(sim::color, 250), 450, -90);

    std::cout << "Reset to factory settings: done" << std::endl;
}
} // namespace sim

int main(int argc, char* argv[])
{
    /* Create the message bus. */
    condvar_wrapper_t bus_sync = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};
    messagebus_init(&bus, &bus_sync, &bus_sync);

    simulation_init();
    sim::reset();

    static TOPIC_DECL(state_topic, RobotState);
    messagebus_advertise_topic(&bus, &state_topic.topic, "/state");

    if (argc != 2) {
        std::cout << "Usage: simulator y|v" << std::endl;
        return 0;
    }

    if (!strcmp(argv[1], "v")) {
        std::cout << "Playing in violet" << std::endl;
        sim::color = VIOLET;
    } else {
        std::cout << "Playing in yellow" << std::endl;
    }

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
            std::cout << "- reset" << std::endl;
            std::cout << "- pos" << std::endl;
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

        goap::Action<RobotState>* path[sim::max_path_len] = {nullptr};
        static goap::Planner<RobotState, GOAP_SPACE_SIZE> planner;
        int len = planner.plan(sim::state, *goal, sim::actions, sim::action_count, path, sim::max_path_len);
        std::cout << "Found a path of length " << std::to_string(len) << " to achieve the " << line << " goal" << std::endl;
        messagebus_topic_publish(&state_topic.topic, &sim::state, sizeof(sim::state));
        for (int i = 0; i < len; i++) {
            bool success = path[i]->execute(sim::state);
            messagebus_topic_publish(&state_topic.topic, &sim::state, sizeof(sim::state));
            if (success == false) {
                std::cout << "Failed to execute action #" << std::to_string(i) << std::endl;
                break; // Break on failure
            }
        }
    }
    return 0;
}
