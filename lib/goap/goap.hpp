/** Action planner for basic AI purposes
 *
 * This file implements the technique presented in "Three States and a Plan:
 * The A.I. of F.E.A.R." by Jeff Orkin.
 */
#ifndef GOAP_HPP
#define GOAP_HPP

#include <cstdint>

namespace goap {

template<typename State>
class Action {
public:
    /** Checks if the given state allows this action to proceed. */
    virtual bool can_run(State state) = 0;

    /** Plans the effects of this action on the state and returns the
     * modified state. */
    virtual State plan_effects(State state) = 0;

    /** Tries to execute the task and returns true if it suceeded. */
    virtual bool execute(State &state) = 0;
};

template<typename State>
class Goal {
public:
    /** Checks if the goal is reached for the given state. */
    virtual bool is_reached(State state) = 0;
};

template<typename State>
class Planner {
    Action<State> **actions;
    size_t action_count;

public:

    /** Constructor, takes an array of possible actions. */
    Planner(Action<State> *actions[], size_t action_count)
        : actions(actions), action_count(action_count)
    {
    }

    /** Finds a plan from state to goal and returns its length.
     *
     * If path is given, then the found path is stored there.
     */
    int plan(State state, Goal<State> &goal,
             Action<State> **path = nullptr, int path_len = 10)
    {
        /* If the goal is reached, then we can abort. */
        if (goal.is_reached(state)) {
            return 0;
        }

        /* If we are not allowed to go further, then abort. */
        if (path_len == 0) {
            return -1;
        }

        int min_cost = 10000;
        Action<State> *best_action = nullptr;

        for (size_t i = 0; i < action_count; i++) {

            /* If the action cannot be used, then abort. */
            if (actions[i]->can_run(state) == false) {
                continue;
            }

            State next_state = actions[i]->plan_effects(state);

            /* Explore this action, depth first.
             *
             * We first do it without a place to store the path, in case this
             * leads nowhere. */
            int res = plan(next_state, goal, nullptr, path_len - 1);

            /* If we found a plan, then use it. */
            if (res >= 0) {

                /* If the cost is higher that what we found so far, abort. */
                if (res < min_cost) {
                    best_action = actions[i];
                    min_cost = res;
                }
            }
        }

        /* If we did not find an action, abort */
        if (best_action == nullptr) {
            return -1;
        }

        if (path != nullptr) {
            /* First get the path, then prepends our solution. */
            State next_state = best_action->plan_effects(state);
            plan(next_state, goal, &path[1], path_len - 1);
            path[0] = best_action;
        }

        return min_cost + 1;
    }
};
};

#endif
