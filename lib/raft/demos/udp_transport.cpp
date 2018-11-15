#include <error/error.h>
#include "udp_transport.hpp"

int make_receive_socket(int port)
{
    auto recv_socket = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(recv_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ERROR("Could not bind socket");
    }


    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    setsockopt(recv_socket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    return recv_socket;
}

