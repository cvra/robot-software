#ifndef GOAP_INTERNALS_HPP
#define GOAP_INTERNALS_HPP

#include <limits>

namespace goap {

template <typename State>
class Action;

template <typename State>
struct VisitedState {
    int priority;
    int cost;
    State state;

    // Used to reconstruct the path
    VisitedState<State>* parent;
    Action<State>* action;

    // Only used for linked list management
    VisitedState<State>* next;
};

template <typename State>
void visited_states_array_to_list(VisitedState<State>* nodes, int len)
{
    for (auto i = 0; i < len - 1; i++) {
        nodes[i].next = &nodes[i + 1];
    }
    nodes[len - 1].next = nullptr;
}

/** Returns the node element in the list with the minimal priority */
template <typename State>
VisitedState<State>* priority_list_pop(VisitedState<State>*& list)
{
    VisitedState<State>* min_elem = nullptr;
    auto min_prio = std::numeric_limits<int>::max();

    // Find the element with the lowest priority
    for (auto p = list; p != nullptr; p = p->next) {
        if (p->priority < min_prio) {
            min_elem = p;
            min_prio = p->priority;
        }
    }

    // Remove it from the list
    if (min_elem == list) {
        list = list->next;
    } else {
        for (auto p = list; p != nullptr; p = p->next) {
            if (p->next == min_elem) {
                p->next = p->next->next;
                min_elem->next = nullptr;
            }
        }
    }

    return min_elem;
}

template <typename State>
VisitedState<State>* list_pop_head(VisitedState<State>*& head)
{
    auto ret = head;
    head = head->next;
    ret->next = nullptr;
    return ret;
}

template <typename State>
void list_push_head(VisitedState<State>*& head, VisitedState<State>* elem)
{
    elem->next = head;
    head = elem;
}

template <typename State>
void update_queued_state(VisitedState<State>* previous, const VisitedState<State>* current)
{
    if (previous->cost > current->cost) {
        previous->priority = current->cost;
        previous->priority = current->priority;
        previous->parent = current->parent;
        previous->action = current->action;
    }
}

} // namespace goap

#endif
