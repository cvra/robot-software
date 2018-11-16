#include "../raft.hpp"
#include "messages_comparator.hpp"
#include <cstring>
#include <cstdio>

bool RaftMessageComparator::isEqual(const void* object1, const void* object2)
{
    using MessageType = TestMessage::Type;
    auto m1 = static_cast<const TestMessage *>(object1);
    auto m2 = static_cast<const TestMessage *>(object2);

    if (m1->type != m2->type) {
        return false;
    }

    if (m1->term != m2->term) {
        return false;
    }

    if (m1->from_id != m2->from_id) {
        return false;
    }

    switch (m1->type) {
        case MessageType::VoteReply:
            return !std::memcmp(&m1->vote_reply, &m2->vote_reply, sizeof(m1->vote_reply));

        case MessageType::VoteRequest:
            return !std::memcmp(&m1->vote_request, &m2->vote_request, sizeof(m1->vote_request));

        case MessageType::AppendEntriesRequest:
            return !std::memcmp(&m1->append_entries_request,
                                &m2->append_entries_request,
                                sizeof(m1->append_entries_request));
            break;

        case MessageType::AppendEntriesReply:
            return !std::memcmp(&m1->append_entries_reply,
                                &m2->append_entries_reply,
                                sizeof(m1->append_entries_reply));
            break;

    }

    return true;
}

SimpleString RaftMessageComparator::valueToString(const void* object)
{
    char buffer[256];
    auto msg = static_cast<const TestMessage *>(object);
    using MessageType = TestMessage::Type;

    buffer[0] = 0;

    switch (msg->type) {
        case MessageType::VoteRequest:
            std::sprintf(buffer,
                         "VoteRequest(term=%d, candidate=%d, last_log_term=%d, "
                         "last_log_index=%d)",
                         msg->term, msg->from_id,
                         msg->vote_request.last_log_term,
                         msg->vote_request.last_log_index);
            break;

        case MessageType::VoteReply:
            std::sprintf(buffer,
                         "VoteReply(term=%d, vote_granted=%d)",
                         msg->term, msg->vote_reply.vote_granted);

            break;

        case MessageType::AppendEntriesRequest:
            std::sprintf(buffer,
                         "AppendEntriesRequest(from=%d, term=%d, count=%d, prevTerm=%d, prevIndex=%d, leaderCommit=%d)",
                         msg->from_id,
                         msg->term,
                         msg->append_entries_request.count,
                         msg->append_entries_request.previous_entry_term,
                         msg->append_entries_request.previous_entry_index,
                         msg->append_entries_request.leader_commit);
            break;

        case MessageType::AppendEntriesReply:
            std::sprintf(buffer,
                         "AppendEntriesReply(from=%d, term=%d, success=%d)",
                         msg->from_id,
                         msg->term,
                         msg->append_entries_reply.success);
            break;
    }
    return buffer;
}

TEST_GROUP(MessageComparatorTestGroup)
{
    RaftMessageComparator cmp;
};

TEST(MessageComparatorTestGroup, WrongTypesReturnFalse)
{
    TestMessage m1, m2;
    m1.type = TestMessage::Type::VoteReply;
    m2.type = TestMessage::Type::VoteRequest;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}

TEST(MessageComparatorTestGroup, WrongTerm)
{
    TestMessage m1;
    TestMessage m2 = m1;
    m2.term ++;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}


TEST(MessageComparatorTestGroup, CompareVoteRequest)
{
    TestMessage m1, m2;
    m1.type = m2.type = TestMessage::Type::VoteRequest;
    m1.from_id = 42;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}

TEST(MessageComparatorTestGroup, StringForVoteRequest)
{
    TestMessage m;
    m.type = TestMessage::Type::VoteRequest;
    m.from_id = 42;
    m.term = 12;
    m.vote_request.last_log_term = 10;
    m.vote_request.last_log_index = 11;

    auto s = cmp.valueToString(&m);

    auto s2 = s.asCharString();

    STRCMP_EQUAL(s2, "VoteRequest(term=12, candidate=42, last_log_term=10, last_log_index=11)");
}

TEST(MessageComparatorTestGroup, StringForVoteReply)
{
    TestMessage m;
    m.type = TestMessage::Type::VoteReply;
    m.term = 12;
    m.vote_reply.vote_granted = true;

    auto s = cmp.valueToString(&m);

    auto s2 = s.asCharString();

    STRCMP_EQUAL(s2, "VoteReply(term=12, vote_granted=1)");
}


SimpleString StringFrom(TestMessage::Type type)
{
    using Type = TestMessage::Type;
    switch (type) {
        case Type::VoteReply:
            return "VoteReply";

        case Type::VoteRequest:
            return "VoteRequest";

        case Type::AppendEntriesRequest:
            return "AppendEntriesRequest";

        case Type::AppendEntriesReply:
            return "AppendEntriesReply";
    }

    return "<unknown>";
}

SimpleString StringFrom(raft::NodeState state)
{
    using State = raft::NodeState;
    switch (state) {
        case State::Candidate:
            return "Candidate";

        case State::Follower:
            return "Follower";

        case State::Leader:
            return "Leader";
    }
    return "<unknown>";
}
