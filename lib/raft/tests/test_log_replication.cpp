#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "test_state_machine.hpp"
#include "messages_comparator.hpp"

class DummyPeer : public TestPeer
{
    virtual void send(const TestMessage &msg)
    {
    }
};

TEST_GROUP(LogReplicationTestGroup)
{
    DummyPeer peer;
    TestPeer *peers[1] = {&peer};
    TestRaftState state{42, peers, 1};
    RaftMessageComparator cmp;

    void setup()
    {
        peer.id = 42;
        mock().installComparator("raft::Message", cmp);
    }

    bool replicate(TestStateMachine::Operation operation, raft::Term term, raft::Index index, raft::Term previous_entry_term, raft::Index previous_entry_index)
    {
        TestMessage msg;
        raft::LogEntry<TestStateMachine::Operation> entry;

        entry.term = term;
        entry.index = index;
        entry.operation = operation;

        msg.type = TestMessage::Type::AppendEntriesRequest;
        msg.append_entries_request.leader_commit = 0;
        msg.append_entries_request.previous_entry_term = previous_entry_term;
        msg.append_entries_request.previous_entry_index = previous_entry_index;
        msg.append_entries_request.entries[0] = entry;
        msg.append_entries_request.count = 1;

        TestMessage reply;
        auto replied = state.process(msg, reply);

        CHECK_TRUE(replied);
        CHECK_TRUE(reply.type == TestMessage::Type::AppendEntriesReply);

        return reply.append_entries_reply.success;
    }

    void send_commit(raft::Index commit_index)
    {
        TestMessage msg;

        msg.type = TestMessage::Type::AppendEntriesRequest;
        msg.append_entries_request.count = 0;
        msg.append_entries_request.previous_entry_term = 0;
        msg.append_entries_request.previous_entry_index = 0;
        msg.append_entries_request.leader_commit = commit_index;

        TestMessage reply;
        auto replied = state.process(msg, reply);

        CHECK_TRUE(replied);
        CHECK_TRUE(reply.type == TestMessage::Type::AppendEntriesReply);
    }
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

TEST(LogReplicationTestGroup, EntryIsReplicatedToAllPeers)
{
    state.become_leader();
    state.replicate(TestStateMachine::Operation::BAR);

    TestMessage expected_msg;
    expected_msg.type = TestMessage::Type::AppendEntriesRequest;
    expected_msg.term = state.term;
    expected_msg.from_id = state.id;
    expected_msg.append_entries_request.entries[0] = state.log[0];
    expected_msg.append_entries_request.count = 1;
    expected_msg.append_entries_request.leader_commit = 0;

    mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &expected_msg);

    // Tick until the heatbeat timer is fired
    while (state.heartbeat_timer > 1) {
        state.tick();
    }
    state.tick();

    mock().checkExpectations();
}

TEST(LogReplicationTestGroup, ProcessAppendEntries)
{
    const auto operation = TestStateMachine::Operation::BAR;

    for (auto i = 0; i < 2; i++) {
        auto success = replicate(operation, 0, i + 1, 0, i);
        CHECK_TRUE(success);
    }

    // Check that the log was correctly modified
    CHECK_EQUAL(2, state.log.size());
    for (auto i = 0; i < 2; i++) {
        CHECK_EQUAL(i + 1, state.log[i].index);
    }
}

TEST(LogReplicationTestGroup, NewEntriesAreRejectedIfRequestComesFromOlderTerm)
{
    // AppendEntries must be rejected if the leader's term is older than ours.
    // See section 5.1 in the raft paper.
    const auto term = 1;
    state.term = 10;
    const auto operation = TestStateMachine::Operation::BAR;
    auto success = replicate(operation, term, 1, 0, 0);
    CHECK_FALSE(success);
    CHECK_EQUAL(0, state.log.size());
}

TEST(LogReplicationTestGroup, EntryIsCommitedIfLeaderAsksSo)
{
    const auto operation = TestStateMachine::Operation::BAR;
    CHECK_EQUAL(0, state.commit_index);
    replicate(operation, 1, 1, 0, 0);
    send_commit(1);
    CHECK_EQUAL(1, state.commit_index);
}

TEST(LogReplicationTestGroup, DoNotCommitEntriesNotPresentInTheLocalLog)
{
    const auto operation = TestStateMachine::Operation::BAR;
    replicate(operation, 1, 1, 0, 0);

    send_commit(2);

    CHECK_EQUAL(1, state.commit_index);
}

TEST(LogReplicationTestGroup, AppendEntriesAreRejectedIfPreviousLogEntryDoesNotExist)
{
    // AppendEntries request should be rejected if the previous entry indicated
    // in the request cannot be found in the local log.
    // See section 5.3 of the paper for details
    const auto operation = TestStateMachine::Operation::BAR;
    replicate(operation, 1, 1, 0, 0);
    auto success = replicate(operation, 1, 1, 1, 3);
    CHECK_FALSE(success);
}

TEST(LogReplicationTestGroup, EntriesAreAppendedToTheLogIfPreviousLogEntryExists)
{
    replicate(TestStateMachine::Operation::BAR, 1, 1, 0, 0);
    auto success = replicate(TestStateMachine::Operation::FOO, 1, 2, 1, 1);

    CHECK_TRUE(success);
    CHECK_EQUAL(2, state.log.size());
    CHECK_TRUE(TestStateMachine::Operation::FOO == state.log[1].operation);
}

TEST(LogReplicationTestGroup, MostRecentTermWinsInCaseThereisATieWithIndex)
{
    // Checks that if two entries conflict (have the same index), then the one
    // with the most recent term wins.
    // See section 5.3

    auto success = replicate(TestStateMachine::Operation::BAR, 1, 1, 0, 0);
    success &= replicate(TestStateMachine::Operation::BAR, 1, 2, 1, 1);
    success &= replicate(TestStateMachine::Operation::FOO, 2, 2, 1, 1);

    CHECK_TRUE(success);
    CHECK_EQUAL(2, state.log.size());
    CHECK_TRUE(TestStateMachine::Operation::FOO == state.log[1].operation);
}

TEST(LogReplicationTestGroup, DuplicateLogEntriesAreIgnored)
{
    TestMessage msg;
    raft::LogEntry<TestStateMachine::Operation> entry[2];

    entry[0].term = 1;
    entry[0].index = 1;
    entry[0].operation = TestStateMachine::Operation::FOO;
    entry[1].term = 1;
    entry[1].index = 2;
    entry[1].operation = TestStateMachine::Operation::BAR;

    msg.type = TestMessage::Type::AppendEntriesRequest;
    msg.append_entries_request.entries[0] = entry[0];
    msg.append_entries_request.entries[1] = entry[1];

    {
        // First, send the message with only one entry
        TestMessage reply;
        msg.append_entries_request.count = 1;
        state.process(msg, reply);
    }

    {
        // Then send the two entries, meaning the first entry will be
        // duplicated
        TestMessage reply;
        msg.append_entries_request.count = 2;
        state.process(msg, reply);
    }

    CHECK_EQUAL(2, state.log.size());
    CHECK_TRUE(TestStateMachine::Operation::BAR == state.log[1].operation);
}

TEST(LogReplicationTestGroup, MatchIndexIsInitializedCorrectly)
{
    raft::LogEntry<TestStateMachine::Operation> entry;
    entry.index= 10;
    state.log.append(entry);

    state.become_leader();
    CHECK_EQUAL(0, peer.match_index);
    CHECK_EQUAL(10, peer.next_index);
}

TEST(LogReplicationTestGroup, UpdatePerPeerIndices)
{
    state.become_leader();
    state.replicate(TestStateMachine::Operation::FOO);
    state.replicate(TestStateMachine::Operation::BAR);

    TestMessage msg, reply;
    msg.type = TestMessage::Type::AppendEntriesReply;
    msg.append_entries_reply.last_index = 1;
    msg.from_id = peer.id;

    {
        // First a successful reply to the first append entry request
        msg.append_entries_reply.success = true;
        auto replied = state.process(msg, reply);
        CHECK_FALSE(replied);
        CHECK_EQUAL(1, peer.match_index);
        CHECK_EQUAL(2, peer.next_index);
    }

    {
        // Then a non succesful entry
        msg.append_entries_reply.success = false;
        auto replied = state.process(msg, reply);
        CHECK_FALSE(replied);
        CHECK_EQUAL(1, peer.match_index);
        // next_index was decremented
        CHECK_EQUAL(1, peer.next_index);
    }
}

TEST(LogReplicationTestGroup, NonReplicatedEntriesAreSentAgainOnHeartbeat)
{
    state.become_leader();
    state.replicate(TestStateMachine::Operation::FOO);
    state.replicate(TestStateMachine::Operation::BAR);
    state.replicate(TestStateMachine::Operation::FOO);

    // Acknowledge only the first operation
    {
        TestMessage msg, reply;
        msg.append_entries_reply.last_index = 1;
        msg.from_id = peer.id;
        msg.type = TestMessage::Type::AppendEntriesReply;
        msg.append_entries_reply.success = true;
        state.process(msg, reply);
    }

    {
        // Since the second log entry was not acknowledged it should be sent
        // again during the heartbeat
        TestMessage expected_msg;
        expected_msg.type = TestMessage::Type::AppendEntriesRequest;
        expected_msg.term = state.term;
        expected_msg.from_id = state.id;
        expected_msg.append_entries_request.entries[0] = state.log[1];
        expected_msg.append_entries_request.entries[1] = state.log[2];
        expected_msg.append_entries_request.count = 2;

        // We note that the first entry has been committed.
        expected_msg.append_entries_request.leader_commit = 1;

        mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &expected_msg);

        // Tick until the heatbeat timer is fired
        while (state.heartbeat_timer > 1) {
            state.tick();
        }
        state.tick();

        mock().checkExpectations();
    }
}

TEST(LogReplicationTestGroup, UpdateCommitIndexWhenMajorityReplicatedIt)
{
    DummyPeer p2;
    TestPeer *peers[] = {&peer, &p2};
    TestRaftState state{42, peers, 2};
    state.become_leader();
    state.replicate(TestStateMachine::Operation::FOO);
    state.replicate(TestStateMachine::Operation::BAR);

    // Acknowledge the first message on one server -> there is majority
    {
        TestMessage msg, reply;
        msg.append_entries_reply.last_index = 1;
        msg.from_id = peer.id;
        msg.type = TestMessage::Type::AppendEntriesReply;
        msg.append_entries_reply.success = true;
        state.process(msg, reply);
    }

    // Since the first entry was replicated on a majority of machines, it can
    // safely be marked as commited
    CHECK_EQUAL(1, state.commit_index);
}

TEST(LogReplicationTestGroup, EntriesFromPreviousTermsCannotBeCommited)
{
    // Entries from previous terms can never be commited explicitely
    // This is needed to ensure correctness. See section 5.4 of the raft paper
    // for explanations
    state.become_leader();

    // Create two entries at term N
    state.replicate(TestStateMachine::Operation::FOO);
    state.replicate(TestStateMachine::Operation::BAR);

    // Then move to term N + 1
    state.term += 1;

    // Acknowledge the first message on one server -> there is majority
    {
        TestMessage msg, reply;
        msg.append_entries_reply.last_index = 1;
        msg.from_id = peer.id;
        msg.type = TestMessage::Type::AppendEntriesReply;
        msg.append_entries_reply.success = true;
        state.process(msg, reply);
    }

    // However the commit_index should not have moved
    CHECK_EQUAL(0, state.commit_index);
}
