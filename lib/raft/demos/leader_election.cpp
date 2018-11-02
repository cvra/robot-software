#include <iostream>
#include <cstdlib>
#include <memory>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdarg>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "error/error.h"
#include "../raft.hpp"

using namespace std;

struct UDPPeer : public raft::Peer {
    int src_id;
    int port;
    int peer_socket;
    struct sockaddr_in servaddr;

    UDPPeer(int src_id, int port) : src_id(src_id), port(port)
    {
        peer_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (!peer_socket) {
            ERROR("Cannot open socket to peer %d", port);
        }

        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        inet_aton("127.0.0.1", &servaddr.sin_addr);
    }

    void send(const raft::Message &msg)
    {
        char buf[sizeof(raft::Message) + sizeof(src_id)];
        memcpy(&buf[0], &src_id, sizeof(src_id));
        memcpy(&buf[sizeof(src_id)], &msg, sizeof(msg));
        sendto(peer_socket, buf, sizeof(buf), 0,
               (struct sockaddr *)(&servaddr), sizeof(servaddr));
    }
};

void mylog(struct error *e, ...)
{
    va_list va;
    va_start(va, e);

    printf("%s:%d: %s: ", e->file, e->line, error_severity_get_name(e->severity));
    vprintf(e->text, va);
    printf("\n");

    va_end(va);
}

void register_error_handlers()
{
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);
    error_register_debug(mylog);
}

std::vector<UDPPeer> make_peers(int my_port, char **argv, int argc)
{
    std::vector<UDPPeer> peers;
    for (auto i = 2; i < argc; i++) {
        auto port = atoi(argv[i]);
        peers.emplace_back(my_port, port);
    }

    return peers;
}

int make_receive_socket(int port)
{
    auto recv_socket = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(recv_socket , (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ERROR("Could not bind socket");
    }


    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    setsockopt(recv_socket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    return recv_socket;
}

bool read_from_socket(int socket, raft::Message &msg, int &addr)
{
    struct sockaddr_in si_other;
    socklen_t slen;
    char buf[sizeof(raft::Message) + sizeof(int)];

    auto recv_len = recvfrom(socket, buf, sizeof(buf), 0,
            (struct sockaddr *) &si_other, &slen);

    if (recv_len < 0) {
        // Timeout occured
        return false;
    } else if (recv_len != sizeof(raft::Message) + sizeof(int)) {
        ERROR("Invalid size %d", recv_len);
        return false;
    }

    memcpy(&msg, &buf[sizeof(int)], sizeof(raft::Message));
    memcpy(&addr, &buf[0], sizeof(int));

    return true;
}

int main(int argc, char **argv)
{
    register_error_handlers();

    if (argc < 3) {
        ERROR("Usage: %s my_port peer1_port [...] peerN_port", argv[0]);
        exit(1);
    }

    const auto my_port = atoi(argv[1]);

    auto peers = make_peers(my_port, argv, argc);

    vector<raft::Peer *> peers_ptrs;
    for (auto &p : peers) {
        peers_ptrs.push_back(&p);
    }

    auto my_socket = make_receive_socket(atoi(argv[1]));

    raft::State state(my_port, peers_ptrs.data(), peers_ptrs.size());

    while (true) {
        state.tick();
        std::this_thread::sleep_for(10ms);
        raft::Message msg;
        int addr;

        if (state.node_state == raft::NodeState::Leader) {
            WARNING("ima leader");
        }

        if (read_from_socket(my_socket, msg, addr)) {
            raft::Message reply;
            auto replied = state.process(msg, reply);
            DEBUG("msg port = %d", addr);

            if (!replied) {
                continue;
            }

            for (auto p : peers) {
                if (p.port == addr) {
                    DEBUG("replying to %d", p.port);
                    p.send(reply);
                }
            }
        }
    }
}
