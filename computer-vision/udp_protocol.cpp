#include <sys/socket.h>
#include "error/error.h"
#include "udp_protocol.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "protobuf/protocol.pb.h"

void udp_send_result(std::string dst, int port, ComputerVisionResult result)
{
    DEBUG("Sending result to %s:%d:\n%s", dst.c_str(), port, result.DebugString().c_str());

    int fd;
    std::string msg;

    if (!result.has_weather_vane_orientation()) {
        return;
    }

    if (!result.SerializeToString(&msg)) {
        ERROR("Could not serialize result message");
        return;
    }

    struct sockaddr_in si;
    memset(&si, 0, sizeof(si));
    si.sin_family = AF_INET;
    si.sin_port = htons(port);

    if (inet_aton(dst.c_str(), &si.sin_addr) == 0) {
        ERROR("Invalid destination %s", dst.c_str());
        return;
    }

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ERROR("cannot create socket");
        return;
    }

    TopicHeader header;
    // TODO do not hardcode those
    header.set_msgid(16);
    header.set_name("/camera");

    std::string header_encoded;
    header.SerializeToString(&header_encoded);

    MessageSize header_size, msg_size;
    header_size.set_bytes(header_encoded.size());
    msg_size.set_bytes(msg.size());

    std::string msg_size_encoded, header_size_encoded;
    header_size.SerializeToString(&header_size_encoded);
    msg_size.SerializeToString(&msg_size_encoded);

    msg = header_size_encoded + header_encoded + msg_size_encoded + msg;
    sendto(fd, msg.data(), msg.size(), 0, (struct sockaddr*)&si, sizeof(si));
    close(fd);
}
