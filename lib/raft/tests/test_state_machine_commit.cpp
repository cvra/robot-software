#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "../raft.hpp"

class MockCommitMachine {
public:
    enum class Operation {
        ZERO = 0,
        ONE,
    };

    void apply(Operation op)
    {
        auto o = static_cast<int>(op);
        mock().actualCall("commit").withParameter("operation", o);
    }
};

using TestMessage = raft::Message<MockCommitMachine>;

TEST_GROUP (StateMachineCommitTestGroup) {
    MockCommitMachine fsm;
    raft::State<MockCommitMachine> state{fsm, 42, nullptr, 0};

    void replicate(MockCommitMachine::Operation operation, raft::Index commit = 0)
    {
        TestMessage msg;
        raft::LogEntry<MockCommitMachine::Operation> entry;

        entry.index = state.log.last_index() + 1;
        entry.term = state.term;
        entry.operation = operation;

        msg.type = TestMessage::Type::AppendEntriesRequest;
        msg.append_entries_request.leader_commit = commit;
        msg.append_entries_request.previous_entry_term = state.log.last_index();
        msg.append_entries_request.previous_entry_index = state.log.last_term();
        msg.append_entries_request.entries[0] = entry;
        msg.append_entries_request.count = 1;

        TestMessage reply;
        state.process(msg, reply);
    }
};

TEST(StateMachineCommitTestGroup, EntriesGetCommited)
{
    replicate(MockCommitMachine::Operation::ONE);

    // The first entry will now be commited
    mock().expectOneCall("commit").withParameter("operation", static_cast<int>(MockCommitMachine::Operation::ONE));

    replicate(MockCommitMachine::Operation::ZERO, 1);
}

TEST(StateMachineCommitTestGroup, OnlyCommitOnceEveryEntry)
{
    mock().expectOneCall("commit").withParameter("operation", static_cast<int>(MockCommitMachine::Operation::ONE));
    mock().expectOneCall("commit").withParameter("operation", static_cast<int>(MockCommitMachine::Operation::ZERO));

    replicate(MockCommitMachine::Operation::ONE);
    replicate(MockCommitMachine::Operation::ZERO, 1);
    replicate(MockCommitMachine::Operation::ONE, 2);
}
