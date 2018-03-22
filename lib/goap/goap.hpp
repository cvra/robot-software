/** Action planner for basic AI purposes
 *
 * This file implements the technique presented in "Three States and a Plan:
 * The A.I. of F.E.A.R." by Jeff Orkin.
 */
#ifndef GOAP_HPP
#define GOAP_HPP

#include <cstdint>
#include <cstdlib>
#include "goap_internals.hpp"

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
    virtual bool is_reached(const State &state) const
    {
        return distance_to(state) == 0;
    }

    /** Computes the distance from state to goal. */
    virtual int distance_to(const State &state) const = 0;
};

template<typename State, int N = 100>
class Planner {
    Action<State> **actions;
    size_t action_count;
    VisitedState<State> nodes[N];

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
        visited_states_array_to_list(nodes, N);

        auto free_nodes = &nodes[0];

        auto open = list_pop_head(free_nodes);
        open->state = state;
        open->cost = 0;
        open->priority = 0;
        open->parent = nullptr;
        open->action = nullptr;

        while (open) {
            auto current = priority_list_pop(open);

            if (goal.is_reached(current->state)) {
                // TODO provide something better for path, maybe iterator like ?
                auto len = 0;
                for (auto p = current->parent; p; p = p->parent) {
                    len ++;
                }

                auto i = len - 1;
                for (auto p = current; p->parent; p = p->parent) {
                    if (path) {
                        path[i] = p->action;
                    }
                    i --;
                }

                return len;
            }

            for (auto i = 0u; i < action_count; i ++) {
                auto action = actions[i];

                if (action->can_run(current->state)) {
                    auto neighbor = list_pop_head(free_nodes);
                    // TODO If neighbor is nullptr, abort

                    neighbor->state = action->plan_effects(current->state);
                    neighbor->cost = current->cost + 1;
                    neighbor->priority = current->priority + 1 + goal.distance_to(neighbor->state);
                    neighbor->parent = current;
                    neighbor->action = action;

                    list_push_head(open, neighbor);
                }

                // TODO: Optimality ?
            }

        }

        // No path was found
        return -1;
    }
};
};

#endif
