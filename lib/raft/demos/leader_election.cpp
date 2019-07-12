#include <iostream>
#include <cstdlib>
#include <memory>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdarg>
#include <thread>

#include <error/error.h>
#include <error_handlers.h>
#include "../raft.hpp"
#include "udp_transport.hpp"

using namespace std;

struct EmptyStateMachine {
    enum class Operation {

    };

    void apply(Operation op)
    {
        (void)op;
    }
};

using Peer = UDPPeer<EmptyStateMachine>;

std::vector<Peer> make_peers(char** argv, int argc)
{
    std::vector<Peer> peers;
    for (auto i = 2; i < argc; i++) {
        auto port = atoi(argv[i]);
        peers.emplace_back(port);
    }

    return peers;
}

int main(int argc, char** argv)
{
    register_error_handlers();

    if (argc < 3) {
        ERROR("Usage: %s my_port peer1_port [...] peerN_port", argv[0]);
        exit(1);
    }

    const auto my_port = atoi(argv[1]);

    auto peers = make_peers(argv, argc);

    vector<raft::Peer<EmptyStateMachine>*> peers_ptrs;
    for (auto& p : peers) {
        peers_ptrs.push_back(&p);
    }

    auto my_socket = make_receive_socket(atoi(argv[1]));

    EmptyStateMachine fsm;
    raft::State<EmptyStateMachine> state(fsm, my_port, peers_ptrs.data(), peers_ptrs.size());

    while (true) {
        state.tick();
        std::this_thread::sleep_for(10ms);
        raft::Message<EmptyStateMachine> msg;

        if (state.node_state == raft::NodeState::Leader) {
            WARNING("ima leader term = %d with %d votes", state.term, state.vote_count);
        }

        if (read_from_socket(my_socket, msg)) {
            raft::Message<EmptyStateMachine> reply;
            DEBUG("msg port = %d", msg.from_id);
            auto replied = state.process(msg, reply);

            if (!replied) {
                continue;
            }

            for (auto p : peers) {
                if (p.id == msg.from_id) {
                    DEBUG("replying to %d", p.id);
                    p.send(reply);
                }
            }
        }
    }
}
