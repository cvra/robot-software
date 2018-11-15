#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

#include "test_state_machine.hpp"

using LogEntry = raft::LogEntry<TestStateMachine::Operation>;

TEST_GROUP(LogOperations)
{
    raft::Log<TestStateMachine::Operation, 10> log;

    LogEntry make_entry(TestStateMachine::Operation op, raft::Term term, raft::Index i)
    {
        LogEntry entry;
        entry.operation = op;
        entry.term = term;
        entry.index = i;
        return entry;
    }
};

TEST(LogOperations, InitialConditions)
{
    CHECK_EQUAL(0, log.size());
}

TEST(LogOperations, AppendEntry)
{
    auto op = TestStateMachine::Operation::BAR;
    LogEntry entry;
    entry.operation = op;
    log.append(entry);

    CHECK_EQUAL(1, log.size());
}

TEST(LogOperations, CanAccessElement)
{
    auto op = TestStateMachine::Operation::BAR;

    LogEntry entry;
    entry.operation = op;
    log.append(entry);
    CHECK_TRUE(op == log[0].operation);
}

TEST(LogOperations, GetLastLogIndex)
{
    LogEntry entry;
    entry.index = 42;
    log.append(entry);
    CHECK_EQUAL(42, log.last_index());
}

TEST(LogOperations, LastIndexOfAnEmptyLogIsZeroByConvention)
{
    CHECK_EQUAL(0, log.last_index());
}

TEST(LogOperations, FindExistingEntry)
{
    raft::Term term = 10;
    raft::Index index = 42;

    LogEntry entry;
    entry.index = index;
    entry.term = term;
    log.append(entry);

    auto ptr = log.find_entry(term, index);
    CHECK_EQUAL(&log[0], ptr);
}

TEST(LogOperations, FindNonExistingEntry)
{
    POINTERS_EQUAL(nullptr, log.find_entry(42, 42));
}

TEST(LogOperations, CanMergeAnEmptyLog)
{
    LogEntry new_entries[1];
    new_entries[0].operation = TestStateMachine::Operation::BAR;
    new_entries[0].index = 1;
    new_entries[0].term = 2;

    log.merge(new_entries, 1);

    CHECK_EQUAL(1, log.size());
}

TEST(LogOperations, MergeNonOverlappingLogs)
{
    log.append(make_entry(TestStateMachine::Operation::BAR, 1, 1));
    LogEntry new_entries[1];
    new_entries[0] = make_entry(TestStateMachine::Operation::FOO, 1, 2);

    log.merge(new_entries, 1);

    CHECK_EQUAL(2, log.size());
    CHECK_TRUE(log[1].operation == TestStateMachine::Operation::FOO);
}

TEST(LogOperations, MergeWhenSomeOfTheNewEntriesAreAlreadyInTheLog)
{
    log.append(make_entry(TestStateMachine::Operation::BAR, 1, 1));
    LogEntry new_entries[2];
    new_entries[0] = log[0];
    new_entries[1] = make_entry(TestStateMachine::Operation::FOO, 1, 2);

    log.merge(new_entries, 2);

    CHECK_EQUAL(2, log.size());
    CHECK_TRUE(log[1].operation == TestStateMachine::Operation::FOO);
}

TEST(LogOperations, MergeConflictingEntries)
{
    // Conflicting entries are defined as entries with the same index.
    // In that case, the entry with the higher term is kept
    log.append(make_entry(TestStateMachine::Operation::BAR, 1, 1));
    LogEntry new_entries[1];
    new_entries[0] = make_entry(TestStateMachine::Operation::FOO, 2, 1);

    log.merge(new_entries, 1);
    CHECK_EQUAL(1, log.size());
    CHECK_EQUAL(2, log[0].term);
}

TEST(LogOperations, EraseAfterEntry)
{
    for (auto i = 0; i < 4; i++) {
        log.append(make_entry(TestStateMachine::Operation::BAR, 1, i));
    }

    log.keep_until(2);
    CHECK_EQUAL(2, log.size());
}
