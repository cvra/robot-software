#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../raft.hpp"
#include "messages_comparator.hpp"

class DummyPeer : public raft::Peer {
    virtual void send(const raft::Message &msg)
    {
        mock().actualCall("send").withParameterOfType("raft::Message", "msg", &msg);
    }
};

raft::Message make_vote_reply(bool vote_granted)
{
    raft::Message msg;
    msg.type = raft::Message::Type::VoteReply;
    msg.vote_reply.vote_granted = vote_granted;

    return msg;
}

raft::Message make_vote_request(raft::Term term, raft::NodeId candidate, unsigned last_log_index, raft::Term last_log_term)
{
    raft::Message msg;
    msg.type = raft::Message::Type::VoteRequest;
    msg.term = term;
    msg.vote_request.candidate = candidate;
    msg.vote_request.last_log_index = last_log_index;
    msg.vote_request.last_log_term = last_log_term;

    return msg;
}

TEST_GROUP(LeaderElectionTestGroup)
{
    DummyPeer peers[2];
    DummyPeer *peers_ptrs[2] = {&peers[0], &peers[1]};
    raft::State state{42, (raft::Peer **)&peers_ptrs[0], 2};
    RaftMessageComparator cmp;

    void setup()
    {
        mock().installComparator("raft::Message", cmp);
    }
};

TEST(LeaderElectionTestGroup, CastsVoteCorrectly)
{
    auto msg = make_vote_request(1, 3, 0, 0);

    auto reply = state.process(msg);
    CHECK_EQUAL(raft::Message::Type::VoteReply, reply.type);
    CHECK_TRUE(reply.vote_reply.vote_granted);
    CHECK_EQUAL(1, reply.term);
}

TEST(LeaderElectionTestGroup, DoesNotVoteForCandidateWithOlderTerm)
{
    state.term = 12;

    auto msg = make_vote_request(1, 3, 0, 0);
    auto reply = state.process(msg);

    CHECK_EQUAL(raft::Message::Type::VoteReply, reply.type);
    CHECK_FALSE(reply.vote_reply.vote_granted);
}

TEST(LeaderElectionTestGroup, UpdatesCurrentTermAfterVoting)
{
    raft::Message msg;
    msg.type = raft::Message::Type::VoteRequest;
    msg.term = 10;

    state.process(msg);
    CHECK_EQUAL(10, state.term);
}

TEST(LeaderElectionTestGroup, CanVoteAtSameTermIfSameCandidate)
{
    auto msg = make_vote_request(10, 42, 0, 0);

    auto reply1 = state.process(msg);
    auto reply2 = state.process(msg);

    CHECK_EQUAL(reply1.type, raft::Message::Type::VoteReply);
    CHECK_TRUE(reply1.vote_reply.vote_granted);
    CHECK_EQUAL(reply2.type, raft::Message::Type::VoteReply);
    CHECK_TRUE(reply2.vote_reply.vote_granted);
}

TEST(LeaderElectionTestGroup, StateStartsAtFollower)
{
    CHECK_EQUAL(state.node_state, raft::NodeState::Follower);
}

TEST(LeaderElectionTestGroup, TransitionsIntoCandidateAtStartOfElection)
{
    mock().ignoreOtherCalls();
    state.start_election();
    CHECK_EQUAL(state.node_state, raft::NodeState::Candidate);
}

TEST(LeaderElectionTestGroup, IncrementsTermAtStartOfElection)
{
    mock().ignoreOtherCalls();
    state.start_election();
    CHECK_EQUAL(1, state.term);
}

TEST(LeaderElectionTestGroup, CastElectionVoteRequests)
{
    auto expected = make_vote_request(1, 42, 0, 0);
    mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &expected);
    mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &expected);

    state.start_election();
    mock().checkExpectations();
}

TEST(LeaderElectionTestGroup, IsElectedOnMajority)
{
    auto m1 = make_vote_reply(true);
    auto m2 = make_vote_reply(true);

    mock().ignoreOtherCalls();
    state.start_election();
    state.process(m1);
    state.process(m2);

    CHECK_EQUAL(state.node_state, raft::NodeState::Leader);
}

IGNORE_TEST(LeaderElectionTestGroup, DoNotCastVotesIfWeAreAlreadyLeader)
{
    FAIL("TODO");
}


TEST(LeaderElectionTestGroup, RestartVotingProcess)
{
    auto m1 = make_vote_reply(true);
    auto m2 = make_vote_reply(true);

    mock().ignoreOtherCalls();
    state.start_election();
    state.process(m1);
    state.start_election();
    state.process(m2);
    CHECK_TRUE(state.node_state != raft::NodeState::Leader);

    state.process(m1);
    CHECK_EQUAL(state.node_state, raft::NodeState::Leader);
}

TEST(LeaderElectionTestGroup, DeniedVotesAreNotCounted)
{
    auto m1 = make_vote_reply(true);
    auto m2 = make_vote_reply(false);
    mock().ignoreOtherCalls();
    state.start_election();
    state.process(m1);
    state.process(m2);
    CHECK_TRUE(state.node_state != raft::NodeState::Leader);
}

TEST(LeaderElectionTestGroup, LeaderSendsHeartBeat)
{
    // First make the node a leader
    auto vote_reply = make_vote_reply(true);
    mock().ignoreOtherCalls();
    state.start_election();
    state.process(vote_reply);
    state.process(vote_reply);

    mock().clear();


    raft::Message msg;
    msg.term = 1;
    msg.type = raft::Message::Type::AppendEntriesRequest;

    mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &msg);
    mock().expectOneCall("send").withParameterOfType("raft::Message", "msg", &msg);

    state.tick();

    mock().checkExpectations();
}

TEST(LeaderElectionTestGroup, LeaderRearmsHeartbeatTimer)
{
    // First make the node a leader
    auto vote_reply = make_vote_reply(true);
    mock().ignoreOtherCalls();
    state.start_election();
    state.process(vote_reply);
    state.process(vote_reply);

    state.tick();

    CHECK_EQUAL(raft::HEARTBEAT_PERIOD - 1, state.heartbeat_timer);
}

TEST(LeaderElectionTestGroup, FollowerDoesNotCareAboutSendingHeartbeat)
{
    state.tick();
}

TEST(LeaderElectionTestGroup, HeartbeatReceptionResetsTimer)
{
    raft::Message msg;
    msg.term = state.term;
    msg.type = raft::Message::Type::AppendEntriesRequest;

    // First lower the election timer so that we can see a difference
    while (state.election_timer > raft::ELECTION_TIMEOUT_MIN) {
        state.tick();
    }

    state.process(msg);

    // Check that the timer was reset
    CHECK(state.election_timer > raft::ELECTION_TIMEOUT_MIN);
    CHECK(state.election_timer < raft::ELECTION_TIMEOUT_MAX);
}

TEST(LeaderElectionTestGroup, ElectionTimerTimeoutStartsElection)
{
    mock().ignoreOtherCalls();

    auto timer_value = state.election_timer;

    for (auto i = 0; i <= timer_value; i++) {
        state.tick();
    }
    CHECK_EQUAL(raft::NodeState::Candidate, state.node_state);
}
