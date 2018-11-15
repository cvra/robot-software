#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>

#include <error/error.h>
#include "../raft.hpp"

template <typename StateMachine>
struct UDPPeer : public raft::Peer<StateMachine> {
    int peer_socket;
    struct sockaddr_in servaddr;

    UDPPeer(int port)
    {
        this->id = port;
        peer_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (!peer_socket) {
            ERROR("Cannot open socket to peer %d", port);
        }

        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        inet_aton("127.0.0.1", &servaddr.sin_addr);
    }

    void send(const raft::Message<StateMachine> &msg)
    {
        sendto(peer_socket, &msg, sizeof(msg), 0,
               (struct sockaddr *)(&servaddr), sizeof(servaddr));
    }
};

template <typename StateMachine>
bool read_from_socket(int socket, raft::Message<StateMachine> &msg)
{
    struct sockaddr_in si_other;
    socklen_t slen;

    auto recv_len = recvfrom(socket, &msg, sizeof(msg), 0,
                             (struct sockaddr *) &si_other, &slen);

    if (recv_len < 0) {
        // Timeout occured
        return false;
    } else if (recv_len != sizeof(raft::Message<StateMachine>)) {
        ERROR("Invalid size %d", recv_len);
        return false;
    }
    return true;
}

int make_receive_socket(int port);
