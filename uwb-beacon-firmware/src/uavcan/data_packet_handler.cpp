#include "main.h"
#include "data_packet_handler.hpp"
#include "ranging_thread.h"
#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/DataPacket.hpp>

using DataPacket = cvra::uwb_beacon::DataPacket;

static void data_packet_cb(const uavcan::ReceivedDataStructure<DataPacket>& msg)
{
    static uint8_t data[1024];

    for (auto i = 0u; i < msg.data.size(); i++) {
        data[i] = msg.data[i];
    }

    ranging_send_data_packet(data, msg.data.size(), msg.dst_addr);
}

int data_packet_handler_init(Node& node)
{
    static uavcan::Subscriber<DataPacket> subscriber(node);

    return subscriber.start(data_packet_cb);
}
