#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include <error/error.h>
#include "../raft.hpp"
#include "udp_transport.hpp"
#include <error_handlers.h>

using namespace std::literals::chrono_literals;

struct EmptyStateMachine {
    using Operation = int;
    void apply(Operation op)
    {
        std::cout << "commited " << op << std::endl;
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

    std::vector<raft::Peer<EmptyStateMachine>*> peers_ptrs;
    for (auto& p : peers) {
        peers_ptrs.push_back(&p);
    }

    auto my_socket = make_receive_socket(atoi(argv[1]));
    EmptyStateMachine fsm;
    raft::State<EmptyStateMachine> state(fsm, my_port, peers_ptrs.data(), peers_ptrs.size());

    bool was_leader = false;
    int previous_log_size = 0;
    raft::Index prev_commit_index = 0;

    while (true) {
        state.tick();
        std::this_thread::sleep_for(10ms);
        raft::Message<EmptyStateMachine> msg;

        if (read_from_socket(my_socket, msg)) {
            raft::Message<EmptyStateMachine> reply;
            auto replied = state.process(msg, reply);

            if (replied) {
                for (auto p : peers) {
                    if (p.id == msg.from_id) {
                        p.send(reply);
                    }
                }
            }

            // Demo application: if we become leader, we append our own ID to
            // the list of nodes
            if (state.node_state == raft::NodeState::Leader && !was_leader) {
                was_leader = true;
                WARNING("replicating node id");
                state.replicate(state.id);
            }

            if (state.log.size() != previous_log_size || state.commit_index != prev_commit_index) {
                for (auto i = 0; i < state.log.size(); i++) {
                    if (state.log[i].index <= state.commit_index) {
                        std::cout << "\e[1m" << state.log[i].operation << ", \e[0m";
                    } else {
                        std::cout << state.log[i].operation << ", ";
                    }
                }
                std::cout << std::endl;
                previous_log_size = state.log.size();
                prev_commit_index = state.commit_index;
            }
        }
    }
}
