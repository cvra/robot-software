#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "test_state_machine.hpp"

TEST_GROUP(LogReplicationTestGroup)
{
    TestRaftState state{42, nullptr, 0};
};

TEST(LogReplicationTestGroup, CanReplicateOperation)
{
    state.replicate(TestStateMachine::Operation::BAR);
    state.replicate(TestStateMachine::Operation::FOO);
    CHECK_TRUE(TestStateMachine::Operation::BAR == state.log[0].operation);
    CHECK_TRUE(TestStateMachine::Operation::FOO == state.log[1].operation);
}

TEST(LogReplicationTestGroup, LogIsAppendedWithCurrentTerm)
{
    state.term = 42;
    state.replicate(TestStateMachine::Operation::BAR);
    CHECK_EQUAL(state.term, state.log[0].term);
}

TEST(LogReplicationTestGroup, LogIndexIncreases)
{
    for (int i = 0; i < 3; i++) {
        state.replicate(TestStateMachine::Operation::BAR);
    }

    for (int i = 0; i < 3; i++) {
        CHECK_EQUAL(i + 1, state.log[i].index);
    }
}

IGNORE_TEST(LogReplicationTestGroup, EntryIsReplicatedToAllPeers)
{
    FAIL("todo");
}

TEST(LogReplicationTestGroup, ProcessAppendEntries)
{
    raft::LogEntry<TestStateMachine::Operation> entry;
    entry.operation = TestStateMachine::Operation::BAR;
    entry.term = 1;
    entry.index = 1;

    TestMessage msg;
    msg.type = TestMessage::Type::AppendEntriesRequest;
    msg.append_entries_request.count = 1;
    msg.append_entries_request.entries[0] = entry;
}
