/** Action planner for basic AI purposes
 *
 * This file implements the technique presented in "Three States and a Plan:
 * The A.I. of F.E.A.R." by Jeff Orkin.
 */
#ifndef GOAP_HPP
#define GOAP_HPP

#include <cstdint>
#include <cstdlib>
#include <goap_internals.hpp>
#include <cstring>

namespace goap {

template <typename State>
class Action {
public:
    /** Checks if the given state allows this action to proceed. */
    virtual bool can_run(const State& state) = 0;

    /** Plans the effects of this action on the state */
    virtual void plan_effects(State& state) = 0;

    /** Tries to execute the task and returns true if it suceeded. */
    virtual bool execute(State& state) = 0;

    virtual ~Action() = default;
};

template <typename State>
class Goal {
public:
    /** Checks if the goal is reached for the given state. */
    virtual bool is_reached(const State& state) const
    {
        return distance_to(state) == 0;
    }

    /** Computes the distance from state to goal. */
    virtual int distance_to(const State& state) const = 0;

    virtual ~Goal() = default;
};

template <typename State, int N = 100>
class Planner {
    VisitedState<State> nodes[N];

public:
    /** Finds a plan from state to goal and returns its length.
     *
     * If path is given, then the found path is stored there.
     */
    int plan(const State& state, Goal<State>& goal, Action<State>* actions[], unsigned action_count, Action<State>** path = nullptr, int path_len = 10)
    {
        visited_states_array_to_list(nodes, N);

        auto free_nodes = &nodes[0];

        auto open = list_pop_head(free_nodes);
        VisitedState<State>* close = nullptr;

        open->state = state;
        open->cost = 0;
        open->priority = 0;
        open->parent = nullptr;
        open->action = nullptr;

        while (open) {
            auto current = priority_list_pop(open);
            list_push_head(close, current);

            if (goal.is_reached(current->state)) {
                auto len = 0;
                for (auto p = current->parent; p; p = p->parent) {
                    len++;
                }

                auto i = len - 1;
                for (auto p = current; p->parent; p = p->parent) {
                    if (path) {
                        if (i < path_len) {
                            path[i] = p->action;
                        }
                    }
                    i--;
                }

                return len;
            }

            for (auto i = 0u; i < action_count; i++) {
                auto action = actions[i];

                if (action->can_run(current->state)) {
                    // Cannot allocate a new node, abort
                    if (free_nodes == nullptr) {
                        // Garbage collect the node that is most unlikely to be
                        // visited (i.e. lowest priority)
                        VisitedState<State>*gc, *gc_prev = nullptr;
                        for (gc = open; gc && gc->next; gc = gc->next) {
                            gc_prev = gc;
                        }

                        if (!gc) {
                            return -2;
                        }

                        if (gc_prev) {
                            gc_prev->next = nullptr;
                        }

                        free_nodes = gc;
                    }

                    auto neighbor = list_pop_head(free_nodes);
                    neighbor->state = current->state;
                    action->plan_effects(neighbor->state);
                    neighbor->cost = current->cost + 1;
                    neighbor->priority = current->priority + 1 + goal.distance_to(neighbor->state);
                    neighbor->parent = current;
                    neighbor->action = action;

                    bool should_insert = true;

                    // Check if the node is already in the list of nodes
                    // scheduled to be visited
                    for (auto p = open; p; p = p->next) {
                        if (p->state == neighbor->state) {
                            should_insert = false;
                            update_queued_state(p, neighbor);
                        }
                    }

                    // Check if the state is in the list of already visited
                    // state
                    for (auto p = close; p; p = p->next) {
                        if (p->state == neighbor->state) {
                            should_insert = false;
                            update_queued_state(p, neighbor);
                        }
                    }

                    if (should_insert) {
                        list_push_head(open, neighbor);
                    } else {
                        list_push_head(free_nodes, neighbor);
                    }
                }
            }
        }

        // No path was found
        return -1;
    }
};

// Distance class, used to build distance metrics that read easily
class Distance {
    int distance;

public:
    Distance shouldBeTrue(bool var)
    {
        distance += var ? 0 : 1;
        return *this;
    }

    Distance shouldBeFalse(bool var)
    {
        distance += var ? 1 : 0;
        return *this;
    }

    Distance shouldBeEqual(int target, int var)
    {
        distance += abs(var - target);
        return *this;
    }

    operator int()
    {
        return distance;
    }
};
}; // namespace goap

#endif
