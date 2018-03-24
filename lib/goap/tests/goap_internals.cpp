#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <array>
#include "../goap_internals.hpp"

using namespace goap;

struct MyState
{
    bool dummy;
};

TEST_GROUP(InternalVisitedListState)
{
    std::array<VisitedState<MyState>, 10> nodes;
};

TEST(InternalVisitedListState, CanConvertToLinkedList)
{
    visited_states_array_to_list<MyState>(nodes.data(), nodes.size());

    for (auto i = 0u; i < nodes.size() - 1; i++) {
        POINTERS_EQUAL(&nodes[i+1], nodes[i].next);
    }
    POINTERS_EQUAL(nullptr, nodes[nodes.size()-1].next);
}

TEST(InternalVisitedListState, CanPopMinimumPriorityEvent)
{
    visited_states_array_to_list<MyState>(nodes.data(), nodes.size());

    for (auto & p : nodes) {
        p.priority = 42;
    }

    nodes[2].priority = 10;
    auto head = nodes.data();
    auto p = priority_list_pop<MyState>(head);

    CHECK_EQUAL(p, &nodes[2]);

    // The element should have been removed from the list
    POINTERS_EQUAL(&nodes[3], nodes[1].next);
    POINTERS_EQUAL(nullptr, p->next);
}

TEST(InternalVisitedListState, CanPopPriorityEventHead)
{
    visited_states_array_to_list<MyState>(nodes.data(), 1);
    auto head = nodes.data();
    priority_list_pop<MyState>(head);

    POINTERS_EQUAL(nullptr, head);
}

TEST(InternalVisitedListState, CanPopFromListHead)
{
    visited_states_array_to_list<MyState>(nodes.data(), nodes.size());
    auto head = nodes.data();

    auto p = list_pop_head<MyState>(head);
    CHECK_EQUAL(&nodes[0], p);
    CHECK_EQUAL(&nodes[1], head);
    POINTERS_EQUAL(nullptr, p->next);
}

TEST(InternalVisitedListState, CanPopLastElement)
{
    visited_states_array_to_list<MyState>(nodes.data(), 1);
    auto head = nodes.data();
    list_pop_head<MyState>(head);
    POINTERS_EQUAL(nullptr, head);
}

TEST(InternalVisitedListState, CanPushListHead)
{
    visited_states_array_to_list<MyState>(nodes.data(), nodes.size());
    VisitedState<MyState> new_elem;
    auto head = nodes.data();

    list_push_head<MyState>(head, &new_elem);
    POINTERS_EQUAL(&new_elem, head);
    POINTERS_EQUAL(&nodes[0], head->next);
}

TEST(InternalVisitedListState, CanPushEmptyList)
{
    VisitedState<MyState> new_elem;
    VisitedState<MyState> *head=nullptr;

    list_push_head<MyState>(head, &new_elem);
    POINTERS_EQUAL(&new_elem, head);
    POINTERS_EQUAL(nullptr, head->next);
}
