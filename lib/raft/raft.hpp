#pragma once

#include <error/error.h>
#include <cstdlib>

namespace raft {

const auto HEARTBEAT_PERIOD = 10;
const auto ELECTION_TIMEOUT_MAX = 500;
const auto ELECTION_TIMEOUT_MIN = 100;

using NodeId = int;
using Term = int;

enum class NodeState {
    Follower=0,
    Candidate,
    Leader
};

struct Message {
    enum class Type {
        VoteRequest=0,
        VoteReply,
        AppendEntriesRequest,
    };

    Type type;
    Term term;

    union {
        struct {
            NodeId candidate;
            unsigned last_log_index;
            Term last_log_term;
        } vote_request;

        struct {
            bool vote_granted;
        } vote_reply;
    };
};

class Peer {
public:
    virtual void send(const Message &msg) = 0;
    virtual ~Peer() {};
};

class State {
public:
    NodeId id;
    Peer **peers;
    int peer_count;
    Term term;
    int vote_count;
    NodeId voted_for;
    NodeState node_state;
    int heartbeat_timer;
    int election_timer;

    State(NodeId id, Peer **peers, int peer_count) : id(id), peers(peers), peer_count(peer_count),
        term(0),
        voted_for(0), node_state(NodeState::Follower), heartbeat_timer(0), election_timer(
            ELECTION_TIMEOUT_MAX)
    {
    }

    bool process(const Message& msg, Message &reply)
    {
        switch (msg.type) {
            case Message::Type::VoteRequest:
                DEBUG("Got a VoteRequest from %d", msg.vote_request.candidate);
                reply.type = Message::Type::VoteReply;
                reply.vote_reply.vote_granted = false;

                if (msg.term > term ||
                    (msg.term == term &&
                     msg.vote_request.candidate == voted_for)) {

                    reply.vote_reply.vote_granted = true;
                    term = msg.term;
                    voted_for = msg.vote_request.candidate;
                    node_state = NodeState::Follower;

                    DEBUG("Granted my vote to %d which has term %d", voted_for, term);
                }

                reply.term = msg.term;
                return true;

            case Message::Type::VoteReply:
                DEBUG("Got a VoteReply(granted = %d)", msg.vote_reply.vote_granted);
                // TODO: How about potential duplicate votes =

                if (node_state != NodeState::Candidate) {
                    break;
                }

                vote_count ++;

                if (msg.vote_reply.vote_granted) {
                    // TODO: Check that this is indeed the majority
                    // it might not be the case if vote_count does not include the
                    // candidate itself.
                    if (2 * vote_count > peer_count) {
                        node_state = NodeState::Leader;
                    }
                } else if (msg.term > term) {
                    term = msg.term;
                    node_state = NodeState::Follower;
                    voted_for = 0;
                    reset_election_timer();
                }


                break;

            case Message::Type::AppendEntriesRequest:
                if (msg.term > term) {
                    node_state = NodeState::Follower;
                }

                reset_election_timer();
                break;
        }

        return false;
    }

    void start_election()
    {
        NOTICE("Starting election...");
        node_state = NodeState::Candidate;
        term ++;
        vote_count = 0;
        voted_for = id;

        Message msg;
        msg.type = Message::Type::VoteRequest;
        msg.term = term;
        msg.vote_request.candidate = id;

        // TODO: fill the following fields
        msg.vote_request.last_log_index = 0;
        msg.vote_request.last_log_term = 0;

        for (auto i = 0; i < peer_count; i++) {
            peers[i]->send(msg);
        }
    }

    void tick()
    {
        if (node_state == NodeState::Leader) {
            tick_heartbeat();
        } else {
            tick_election();
        }
    }

private:
    void tick_heartbeat()
    {
        if (heartbeat_timer > 0) {
            heartbeat_timer --;
        } else {
            DEBUG("Sending heartbeat");
            Message msg;
            msg.type = Message::Type::AppendEntriesRequest;
            msg.term = term;
            for (auto i = 0; i < peer_count; i ++) {
                peers[i]->send(msg);
            }

            // Rearm timer
            heartbeat_timer = HEARTBEAT_PERIOD - 1;
        }
    }

    void tick_election()
    {
        if (election_timer > 0) {
            election_timer --;
        } else {
            start_election();
            reset_election_timer();
        }
    }

    void reset_election_timer()
    {
        auto x = ELECTION_TIMEOUT_MAX - ELECTION_TIMEOUT_MIN;

        election_timer = ELECTION_TIMEOUT_MIN;
        election_timer += std::rand() % x;
    }
};
}
