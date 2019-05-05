#include <string.h>
#include <iostream>

#include "msgbus_port.h"
#include "simulation.h"

#include "strategy.h"
#include "strategy/goals.h"
#include "strategy/state.h"
#include "strategy_impl/actions.h"

namespace sim {
RobotState state;

AcceleratorGoal accelerator_goal;
TakeGoldoniumGoal take_goldenium_goal;
ClassifyBluePucksGoal classify_blue_goal;
goap::Goal<RobotState>* goals[] = {
    &accelerator_goal,
    &take_goldenium_goal,
    &classify_blue_goal,
};
const char* goal_names[] = {
    "accelerator",
    "goldenium",
    "blue",
};
const size_t goal_count = sizeof(goals) / sizeof(goap::Goal<RobotState>*);

enum strat_color_t color = YELLOW;
strategy_context_t* ctx = strategy_simulated_impl(color);

RetractArms retract_arms(ctx);
TakePuck take_pucks[] = {
    {ctx, 0},
    {ctx, 1},
    {ctx, 2},
    {ctx, 3},
    {ctx, 4},
    {ctx, 5},
    {ctx, 6},
    {ctx, 7},
    {ctx, 8},
    {ctx, 9},
    {ctx, 10},
    {ctx, 11},
};
DepositPuck deposit_puck[] = {
    {ctx, 0},
    {ctx, 1},
    {ctx, 2},
    {ctx, 3},
    {ctx, 4},
};
LaunchAccelerator launch_accelerator(ctx);
TakeGoldonium take_goldonium(ctx);
goap::Action<RobotState>* actions[] = {
    &retract_arms,
    &take_pucks[0],
    &take_pucks[1],
    &take_pucks[2],
    &take_pucks[3],
    &take_pucks[4],
    &take_pucks[5],
    &take_pucks[6],
    &take_pucks[7],
    &take_pucks[8],
    &take_pucks[9],
    &take_pucks[10],
    &take_pucks[11],
    &deposit_puck[0],
    &deposit_puck[1],
    &deposit_puck[2],
    &deposit_puck[3],
    &deposit_puck[4],
    &launch_accelerator,
    &take_goldonium,
};
const auto action_count = sizeof(actions) / sizeof(actions[0]);

const int max_path_len = 10;

void reset(void)
{
    sim::state = initial_state();
    sim::state.arms_are_indexed = true;
    sim::ctx->goto_xya(sim::ctx, MIRROR_X(sim::color, 250), 450, MIRROR_A(sim::ctx->color, -90));
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
            for (size_t i = 0; i < sim::goal_count; i++) {
                std::cout << "- " << sim::goal_names[i] << std::endl;
            }
            continue;
        }

        if (!strcmp(line.c_str(), "reset")) {
            sim::reset();
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
            } else {
                std::cout << "Action #" << std::to_string(i) << " succeeded" << std::endl;
            }
        }
    }
    return 0;
}
