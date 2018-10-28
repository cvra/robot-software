#include "../raft.hpp"
#include "messages_comparator.hpp"
#include <cstring>
#include <cstdio>

bool RaftMessageComparator::isEqual(const void* object1, const void* object2)
{
    auto m1 = static_cast<const raft::Message *>(object1);
    auto m2 = static_cast<const raft::Message *>(object2);

    if (m1->type != m2->type) {
        return false;
    }

    if (m1->term != m2->term) {
        return false;
    }

    switch (m1->type) {
        case raft::Message::Type::VoteReply:
            return !std::memcmp(&m1->vote_reply, &m2->vote_reply, sizeof(m1->vote_reply));

        case raft::Message::Type::VoteRequest:
            return !std::memcmp(&m1->vote_request, &m2->vote_request, sizeof(m1->vote_request));
    }

    return true;
}

SimpleString RaftMessageComparator::valueToString(const void* object)
{
    char buffer[256];
    auto msg = static_cast<const raft::Message *>(object);

    switch(msg->type) {
        case raft::Message::Type::VoteRequest:
            std::sprintf(buffer,
                         "VoteRequest(term=%d, candidate=%d, last_log_term=%d, "
                         "last_log_index=%d)",
                         msg->term, msg->vote_request.candidate,
                         msg->vote_request.last_log_term,
                         msg->vote_request.last_log_index);
            break;

        case raft::Message::Type::VoteReply:
            std::sprintf(buffer,
                         "VoteReply(term=%d, vote_granted=%d)",
                         msg->term, msg->vote_reply.vote_granted);

            break;

        case raft::Message::Type::AppendEntriesRequest:
            std::sprintf(buffer, "AppendEntriesRequest(term=%d)", msg->term);
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
    raft::Message m1, m2;
    m1.type = raft::Message::Type::VoteReply;
    m2.type = raft::Message::Type::VoteRequest;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}

TEST(MessageComparatorTestGroup, WrongTerm)
{
    raft::Message m1;
    raft::Message m2 = m1;
    m2.term ++;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}


TEST(MessageComparatorTestGroup, CompareVoteRequest)
{
    raft::Message m1, m2;
    m1.type = m2.type = raft::Message::Type::VoteRequest;
    m1.vote_request.candidate = 42;

    CHECK_FALSE(cmp.isEqual(&m1, &m2));
}


TEST(MessageComparatorTestGroup, StringForVoteRequest)
{
    raft::Message m;
    m.type = raft::Message::Type::VoteRequest;
    m.vote_request.candidate = 42;
    m.term = 12;
    m.vote_request.last_log_term = 10;
    m.vote_request.last_log_index = 11;

    auto s = cmp.valueToString(&m);

    auto s2 = s.asCharString();

    STRCMP_EQUAL(s2, "VoteRequest(term=12, candidate=42, last_log_term=10, last_log_index=11)");
}

TEST(MessageComparatorTestGroup, StringForVoteReply)
{
    raft::Message m;
    m.type = raft::Message::Type::VoteReply;
    m.term = 12;
    m.vote_reply.vote_granted = true;

    auto s = cmp.valueToString(&m);

    auto s2 = s.asCharString();

    STRCMP_EQUAL(s2, "VoteReply(term=12, vote_granted=1)");
}


SimpleString StringFrom(raft::Message::Type type)
{
    using Type = raft::Message::Type;
    switch (type) {
        case Type::VoteReply:
            return "VoteReply";
        case Type::VoteRequest:
            return "VoteRequest";
        case Type::AppendEntriesRequest:
            return "AppendEntriesRequest";
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
