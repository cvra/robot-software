#pragma once

namespace raft {
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
        VoteReply=1,
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
};

class State {
public:
    NodeId id;
    Peer *peers;
    int peer_count;
    Term term;
    int vote_count;
    NodeId voted_for;
    NodeState node_state;

    State(NodeId id, Peer *peers, int peer_count) : id(id), peers(peers), peer_count(peer_count), term(0),
        voted_for(0), node_state(NodeState::Follower)
    {
    }

    Message process(const Message& msg)
    {
        Message reply;

        switch (msg.type) {
            case Message::Type::VoteRequest:
                reply.type = Message::Type::VoteReply;
                reply.vote_reply.vote_granted = false;

                if (msg.term > term ||
                    (msg.term == term &&
                     msg.vote_request.candidate == voted_for)) {

                    reply.vote_reply.vote_granted = true;
                    term = msg.term;
                    voted_for = msg.vote_request.candidate;
                }

                reply.term = msg.term;
                break;

            case Message::Type::VoteReply:
                vote_count ++;

                if (!msg.vote_reply.vote_granted) {
                    break;
                }

                if (vote_count > (peer_count / 2)) {
                    node_state = NodeState::Leader;
                }

                break;
        }

        return reply;
    }

    void start_election()
    {
        node_state = NodeState::Candidate;
        term ++;
        vote_count = 0;

        Message msg;
        msg.type = Message::Type::VoteRequest;
        msg.term = term;
        msg.vote_request.candidate = id;

        // TODO: fill the following forms
        msg.vote_request.last_log_index = 0;
        msg.vote_request.last_log_term = 0;

        for (auto i = 0; i < peer_count; i++) {
            peers[i].send(msg);
        }
    }
};
}
